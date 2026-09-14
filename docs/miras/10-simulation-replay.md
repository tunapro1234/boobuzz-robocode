# 10 — Simülasyon ve Kayıt/Oynatma

> 🎯 **Tuna'nın talebi:** *"ftcde hala bi simülasyon isterim tatlı olur."*

## Özet

Geçen sezon **gerçekten çalışan bir simülasyon** kuruldu: `re-cock-nize`.
JPype ile Python'dan **gerçek Java robot kodunu** çağırıyor, fizik motoruyla
sahayı simüle ediyor. 56 sim logu ve 4 unit test dosyası var — yani çalıştı.

Ayrıca hiç test edilmemiş bir kayıt/oynatma sistemi var: **"Parrot"** (1.120 satır).

---

## [D] SİMÜLASYONDA KOŞTU — `de-cock/re-cock-nize` (74 MB, 50 commit)

**Durum:** Branch `2d-logic-sim-pybullet`, son commit 11 Ara 2025 ("task cancel falan").
⚠️ **Commit edilmemiş büyük bir yeniden yapılandırma var** — dosyalar `java_runner/`
ve `game_sim/` altına taşınmış, staged ama commit'lenmemiş. Devralmadan önce
o değişikliklere bakılmalı.

### Yapı
```
java_runner/          Robot kodunu simüle etme + unit test ortamı   33 py, 8.614 satır
  ftc_sim/
    bridge/           JPype köprüsü — jvm.py, logger.py, settings.py
    physics/          ballistics.py + hareket modeli
    subsystems/       Sim HAL (sahte donanım)
    android_stubs/    Android sınıf stub'ları (SDK'sız JVM için)
    tuning/
  tests/              4 test dosyası
  blue_sim_teleop.py  Klavye → Java RobotAction → Sim HAL
  cartographer_viewer.py
  config.yaml
game_sim/             Gymnasium RL ortamı                            4 py, 197 satır
  train_rl.sh
logs/                 56 adet sim_log_2025121*.json
```

### Nasıl çalışıyor — kritik nokta
`ftc_sim/bridge/jvm.py` gerçek robot-code derleme çıktısını JVM'e yüklüyor:
```python
ROBOT_CODE_ROOT / "TeamCode/build/intermediates/javac/release/compileReleaseJavaWithJavac/classes"
# "direct" mode — robot-code build'ini doğrudan kullan (sadece ./gradlew compile gerekiyor)
# FTC_ROBOT_CODE_ROOT=/path/to/robot-code ile yol verilebiliyor
```
**Yani simülasyon sahte bir kopyayı değil, robota yüklenen kodun aynısını koşturuyor.**
Simülasyonun tek gerçek değeri budur; bu doğru yapılmış.

`android_stubs/` — Android sınıflarının sahte implementasyonları, böylece robot kodu
Android olmadan JVM'de çalışabiliyor.

### Bağlantı noktası: `iface/` paketi (`08-architecture-settings.md`)
```
RobotState  (241)  ← simülasyon dolduruyor (sensörler, pozlar)
RobotAction (62)   ← robot mantığı üretiyor (motor komutları)
RobotIntent (282)  ← üst seviye niyet
TaskStatus  (84)
```
Robot mantığı `RobotState` alıp `RobotAction` döndürüyor; simülasyon HAL'i bunu
uyguluyor. **Donanımdan arındırılmış mantık katmanı olmadan simülasyon kurulamaz.**

> ⚠️ **Bu sezon için en önemli mimari karar:** Simülasyon istiyorsan `iface/` benzeri
> ayrımı **baştan** kur. Sonradan eklemek, tüm subsystem çağrılarını soyutlamak
> demek — çok pahalı.

### Kalibre edilmiş fizik — `java_runner/config.yaml`
Gerçek robota fit edilmiş parametreler:
```yaml
drivetrain:
  mass_kg: 12.0            wheel_diameter_in: 4.0
  stall_torque_nm: 7.5     free_speed_rpm: 350.0
  drive_efficiency: 0.8    strafe_efficiency: 0.75   turn_efficiency: 0.7
  linear_drag: 0.5         brake_drag: 0.6
  brake_decel_ips2: 260.0  brake_turn_decel_dps2: 420.0
  max_speed_ips: 394.0     max_turn_rate_dps: 180.0
  wheel_lateral_friction: 12.0   wheel_spinning_friction: 0.9
  body_linear_damping: 0.7       body_angular_damping: 1.2
  anisotropic_forward: 1.0       anisotropic_strafe: 0.3
collision:
  push_bias: 0.7  velocity_transfer: 0.6  velocity_damping: 0.3
```
**Mecanum'un anizotropik sürtünmesi modellenmiş** (`anisotropic_strafe: 0.3` — yana
kayma ileriden çok daha zor). Bu detay çoğu FTC simülasyonunda yok.

### Kullanım
```bash
pip install -r java_runner/requirements.txt
./java_runner/run_tests.sh                    # unit testler
python java_runner/blue_sim_teleop.py         # klavyeyle sim teleop (pygame UI)
python java_runner/blue_sim_teleop.py --cli   # terminal modu
./game_sim/train_rl.sh sanity 200             # RL sanity check
```

### Testler — `java_runner/tests/`
```
test_java_robot_shooting_manager.py
test_java_shooter_feeder_behavior.py
test_java_sim_harness.py
test_sim_smoke.py
```
**Gerçek Java atış mantığının unit testi.** Robot olmadan "tetiğe basınca ne oluyor"
sorusunu test edebilmek değerli — özellikle `ShootingController`'ın 8 state'li
durum makinesi gibi şeyler için.

### Sim logları — `logs/` 56 dosya
Format: `{version, created_at, config, entries}`. 11 Aralık 2025, saat 02:43–07:37
arası — tek bir gecede yoğun çalışma.

### ⚠️ Pedro 3 uyumu
Simülasyon Pedro 2.x API'sine göre yazıldı. **Pedro 3'te API tamamen değişti**
(`06-drivetrain-motion-auto.md`'deki geçiş tablosu). JVM köprüsünün ve sim HAL'inin
güncellenmesi gerekecek.

---

## [C] "Parrot" kayıt/oynatma — `logic/parrot/` 1.120 satır

| Dosya | Satır | Ne |
|---|---|---|
| `SmartParrot.java` | 352 | Akıllı oynatma |
| `ActionLogger.java` | 229 | Eylemleri kaydet |
| `DumbParrot.java` | 199 | Ham tekrar oynatma |
| `ReplayLoader.java` | 147 | Kayıt yükleme |
| `ParrotSettings.java` | 105 | Ayarlar |
| `ReplayFrame.java` | 51 | Tek kare |
| `ReplayData.java` | 37 | Kayıt kabı |

**Fikir:** Teleop'ta sürücünün yaptıklarını kaydet, otonomda geri oynat.
FTC'de bilinen bir teknik — otonom yazmak yerine "iyi bir tur sür, kaydet".

`DumbParrot` = ham komutları zamanla tekrar oynat.
`SmartParrot` = muhtemelen poz düzeltmeli oynatma (kayma telafisi).

**Kanıt yok.** Ama `re-cock-nize/java_runner/export/blue_0_replay.json` diye bir dosya
git geçmişinde geçiyor — yani en azından simülasyonda denenmiş olabilir.

**Değerlendirme:** Cazip ama riskli. Ham oynatma FTC'de nadiren güvenilir çalışır
(batarya voltajı, sürtünme, başlangıç pozu farkları birikir). `AutoBuilder`
(`06-drivetrain-motion-auto.md`) daha güvenilir yol. **Bu sezon öncelik verme.**

---

## İlgili: `de-cock/ball-auto-istic` fizik simülasyonu

Ayrı bir simülasyon hattı — top balistiği için (`01-shooter.md`):
```
physics/bullet_sim.py     PyBullet 3D fizik
physics/engine.py, models.py, variation.py
tools/sim_3d.py, verify_bounce.py, verify_physics.py
messi/ + ronaldinho/      iki balistik çözücü (RK4, Nelder-Mead)
```
Bu, robot simülasyonu değil **mermi simülasyonu**. İkisi ayrı tutulmuş, doğru.

---

## Bu sezon için öneri

### Eğer simülasyon istiyorsak (Tuna istiyor):

1. **`iface/` ayrımını ilk günden kur.** `RobotState` → mantık → `RobotAction`.
   Bu yapılmazsa simülasyon sonradan eklenemez. **Bu, mimari tartışmasının en kritik
   maddesi** — interface tartışmasından bile önce gelir.

2. **`java_runner` köprüsünü devral, Pedro 3'e güncelle.** `jvm.py`'nin "direct mode"
   yaklaşımı (gerçek build çıktısını yükle) korunmalı — simülasyonun değeri orada.

3. **`android_stubs/` doğrudan taşınır** — SDK sürümünden bağımsız.

4. **`config.yaml` fizik parametrelerini şablon al**, yeni robota göre yeniden fit et.
   Özellikle `anisotropic_strafe` gibi mecanum detaylarını koru.

5. **Unit testlerle başla, görsel simülasyonla değil.** `test_java_shooter_feeder_behavior.py`
   tarzı testler robot olmadan mantık doğrulamanın en ucuz yolu. Pygame UI sonra.

6. **Commit edilmemiş refactor'ü önce çöz.** `re-cock-nize` çalışma ağacında staged
   ama commit'lenmemiş büyük bir yapı değişikliği duruyor.

7. **Parrot'u erteleme listesine koy.** İlginç ama otonom için `AutoBuilder` daha güvenli.

8. **RL tarafını (`game_sim`, 197 satır) şimdilik bırak.** Sanity check'ten öteye
   gitmemiş.

```bash
# Simülasyon deposu
cd /home/shared/projects/archive/ftc/de-cock/re-cock-nize
cat README.md
git status                      # ⚠️ commit edilmemiş refactor
cat java_runner/config.yaml
cat java_runner/ftc_sim/bridge/jvm.py
ls java_runner/tests/

# Robot tarafındaki arayüzler
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git checkout d7711d0^ -- $P/iface/
git show d7711d0^:$P/logic/parrot/SmartParrot.java
```
