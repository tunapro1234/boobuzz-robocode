# 07 — Teleop ve Controller Katmanı

## Özet

Maça giren teleop mimarisi: `Robot` (donanım konteyneri) + 5 controller + 2 OpMode
(Blue/Red). Toplam ~2.000 satır controller + 864 satır teleop.

**Bu katman hem en çok kanıtlanmış hem de en çok teknik borç taşıyan yer.**
Yarışmada çalıştı — ama `if (false && ...)` ölü dalları, controller'lar arası
dairesel bağımlılık ve Blue/Red kopyalaması buradaydı.

---

## [A] MAÇTA KOŞTU

### `contingency/lvbelc5/Robot.java` — 265 satır
Donanım konteyneri. Auto ve teleop ortak kullanıyor.

```java
public final Follower follower;
public final ShooterPidfPowerSubsystem shooter;
public final HoodSubsystem hood;
public final FeederPowerSubsystem feeder;
public final IntakePowerSubsystem intake;
public final TurretPidPazarSubsystem turret;
public final Limelight limelight;
public final LocalizerController localizer;
public final Alliance alliance;
```

İki kurucu: teleop (`PoseStorage`'dan poz alır) ve auto (açık başlangıç pozu).
Poz seçim mantığı:
```java
if (startPose != null)                   pose = startPose;         // auto
else if (PoseStorage.isExplicitlySet())  pose = PoseStorage.get(); // auto'dan devir
else                                     pose = alliance.isRed()   // varsayılan
     ? AutoLocations.Red.START_MISSIONARY : AutoLocations.Blue.START_MISSIONARY;
```

Kalibrasyon API'si: `runCalibration()`, `isCalibrating()`, `getCalibrationProgress()`
(minimum 3 sn üzerinden), `startTeleop()`, `periodic()`, `savePose()`, `stop()`.

> 🚩 **Teknik borç:** `private static final boolean USE_KALMAN = false;`
> 380 satırlık `LocalizerController` yazıldı, sonra hardcoded bayrakla kapatıldı.

**Bu sezon:** Yapı doğru (donanım konteyneri + periodic + kalibrasyon yaşam döngüsü).
~80 satırda yeniden yazılabilir.

### `ShootingController.java` — 585 satır
Atış durum makinesi. 8 state:
```
IDLE · AUTO_SHOOT · WARMUP · FINISHING_PULSE · BURST · JAM_CLEAR
SINGLE_SHOT · SHOOTER_JAM_CLEAR
```

Gamepad eşlemesi (öncelik sırasıyla):
| Giriş | State |
|---|---|
| LB | `JAM_CLEAR` — intake+feeder ters |
| Y (START basılı değilken) | `SHOOTER_JAM_CLEAR` — shooter ileri/geri salınım (500 ms) |
| RB | `AUTO_SHOOT` — zone'u yoksay |
| RT > 0.5 | zone içindeyse `AUTO_SHOOT`, dışındaysa `WARMUP` |
| RT bırakıldı ama pulse sürüyor | `FINISHING_PULSE` |

**İyi tasarlanmış detaylar:**
- `FINISHING_PULSE`: tetik bırakılsa bile feeder darbesi bitene kadar shooter RPM'de
  kalıyor. Yarıda kalan atış yok.
- `WARMUP`: zone dışındayken shooter döner, feeder beslemez. Zone'a girince atış anında.
- **Taret limit uyarısı**: atış deneniyor ama taret hedefe dönemiyorsa `gamepad.rumble(200)`
  sürekli titreşim. Sürücü neden atamadığını hissediyor.
- **RPM timeout kompanzasyonu** (~120 satır): shooter hedefe X saniye ulaşamazsa
  (`FULL_POWER_THRESHOLD=0.95`, `MIN_ERROR_FOR_COMPENSATION=30`) PID'in oturması için
  hedefe offset ekliyor + gamepad titreşimiyle haber veriyor. *Varsayılan kapalı
  (`rpmCompensationEnabled = false`).*

> 🚩 **Teknik borç — bu sezonun en önemli dersi:**
> ```java
> } else if (false && gamepad.right_bumper && solution != null) {  // BURST disabled
> } else if (false && gamepad.y && solution != null) {             // SINGLE_SHOT disabled
> ```
> 8 state'ten 2'si `false &&` ile devre dışı ama `switch` içinde duruyor, execute
> metotları yazılı, yorum blokları içinde başka ölü kod var.
>
> **Kural: kapatılan özellik silinir. `git revert` var.**

### `AimingController.java` — 207 satır
→ `02-turret.md`'de ayrıntılı. Özet: taret+hood+shooter hedefleme, fallback çözümü
(sabit hood 36-52°, RPM 4200-5850), elle offset, `isTurretAtLimit()`.

### `ZoneController.java` — 100 satır ⭐ *temiz örnek*
Robotun atış bölgesinde olup olmadığını izliyor.
```java
enum ZoneMode { RECTANGLE, CIRCLE }
COLOR_IN_ZONE  = {0, 1, 0}   // yeşil
COLOR_OUT_ZONE = {1, 0, 0}   // kırmızı
LED_DURATION_MS = 1000       // her loop yenileniyor
RUMBLE_MS = 300              // zone çıkışında titreşim
```
`justEnteredZone()` / `justExitedZone()` kenar tespiti var.
Gamepad LED'i sürücüye bölge durumunu gösteriyor.

**100 satır, tek sorumluluk, yan etkisi yok.** Controller'ların nasıl olması
gerektiğinin örneği. Bu sezon da benzeri lazım.

### `RecoveryController.java` — 195 satır ⭐ *fikir çok değerli*
**Acil durum modu.** Hedefleme çöktüğünde (vision gitti, lokalizasyon kaçtı) robotu
kullanılabilir tutuyor:
```java
RECOVERY_RPM = 4000.0          // sabit
DEFAULT_HOOD_ANGLE = 45.0      // sabit
DEFAULT_TURRET_ANGLE = 0.0     // kilitli
HOLD_TIME_SEC = 2.0            // BACK 2 sn basılı → aç/kapa
TURRET_STEP = 2.0°  HOOD_STEP = 1.0°   // dpad ile elle ayar
```
Aktifken: taret 0°'de kilitli + dpad ile elle çevirme, hood sabit 45° + dpad,
RPM sabit 4000, sürüş **robot-merkezli** (field-centric değil — çünkü heading'e
güvenilmiyor), atış normal çalışıyor.

**Bu fikri ilk günden koy.** Yarışmada vision/odometri çökünce maçı kurtarır.
2 saniye basılı tutma şartı kazara aktivasyonu engelliyor.

### `TelemetryManager.java` — 517 satır
Gamepad2 ile gezilen robot-içi ayar menüsü.

7 bölüm aç/kapa: `POSE`, `VISION`, `SHOOTER`, `TURRET`, `ZONE_MODE`,
`CALIB_MODEL` (MT2/TAG3D/PIXEL), `RPM_COMP`.

7 canlı ayarlanabilir parametre (isim, varsayılan, min, max, adım, birim tablolarıyla):
```
TURRET_OFFSET (±15°, adım 1)     TURRET_STEP (0.5-10°, adım 0.5)
RPM_WEIGHT (0-1, adım 0.05)      HOOD_WEIGHT (0-1, adım 0.05)
RPM_OFFSET (±500, adım 50)       HOOD_OFFSET (±10°, adım 0.5)
RPM_TIMEOUT (1-10 s, adım 0.5)
```
`applyTo(aiming)` / `applyTo(zone)` / `applyTo(robot)` / `applyTo(shooting)` ile
değerleri dağıtıyor.

**Değerlendirme:** Pit'te laptop olmadan ayar yapabilmek gerçek bir ihtiyaç ve bu
maçta kullanıldı. Ama 517 satır ve **Panels + FTC Dashboard ikisi de aynı işi yapıyor**
(ikisi de yeni projede kurulu). Bu sezon: Dashboard'ın `@Config` anotasyonuyla
çözülebilecek kısmı ona bırak, sadece gerçekten gamepad'den ayarlanması gereken
2-3 parametre için minik bir menü yaz.

### Teleop OpMode'ları — `BlueTeleop.java` / `RedTeleop.java` 219 ×2

Gamepad 1 (sürücü):
```
Sol stick      sürüş           Sağ stick X   dönüş
RT             auto-shoot      RB            burst (→ auto-shoot)
LB             jam clear       LT            elle intake
B (basılı)     PARK'a otomatik sürüş
BACK           recovery modu (2 sn basılı)
START + Y      hard reset (2 sn basılı) — pozu duvara göre sıfırla
dpad ←/→       elle taret offset      dpad ↑/↓   elle hood offset
```
Gamepad 2: tuning menüsü (dpad gez/ayarla, A alt menü, B geri, Y reset).

Init döngüsünde `robot.runCalibration()` + kalibrasyon ekranı; `start()`'ta
`robot.startTeleop()`; sonunda `robot.savePose()`.

> 🚩 **BlueTeleop ↔ RedTeleop farkı: 219 satırda 6 satır.**
> `ALLIANCE` sabiti, telemetri başlığı, `AutoLocations.{Blue,Red}.PARK`,
> stick işaretleri (kırmızıda `-left_stick_y`, `-left_stick_x`), hard reset X koordinatı.
>
> **Bu sezon: tek OpMode + alliance parametresi.**

> 🚩 **Dairesel bağımlılık:**
> ```java
> shooting.setAimingController(aiming);   // taret limit uyarısı için
> aiming.setShootingController(shooting); // jam clear'da hood override için
> ```
> İki controller birbirini setter ile arıyor. Sorumlulukların yanlış yere düştüğünün
> klasik işareti.

---

## [C] TEST EDİLMEDİ

### Eski teleop altyapısı — `teleop/` 361 satır
| Dosya | Satır |
|---|---|
| `AllianceTeleopBase.java` | ~ |
| `GamepadInputMapper.java` | ~ |
| `opmodes/alliance/{Blue,Red}Teleop.java` | ~ |
| `opmodes/alliance/{Blue,Red}TeleopReset.java` | ~ |

**`AllianceTeleopBase` tam olarak doğru fikirdi** — alliance'ı parametreleştiren ortak
taban sınıf. Ama LC5'e geçilirken terk edildi ve kopyala-yapıştıra dönüldü.
Bu sezon bu fikri geri getir.

`GamepadInputMapper` — gamepad eşlemesini koddan ayırma denemesi.

### Eski contingency seviyeleri — 7.537 satır [C]
| Seviye | Satır | Not |
|---|---|---|
| `contingency/level0` | 282 | `Level0EmergencyTeleop` — en temel |
| `contingency/level1` | 300 | `Level1SubsystemTeleop` |
| `contingency/level2` | 2.198 | Robot + teleop + 6 otonom |
| `contingency/level3` | 860 | + AimingController, ShootingController |
| `contingency/level4` | 4.197 | + LocalizerController, RonaldoShEngine, AutoBuilder |
| `contingency/level5` | 6.210 | maça giren sürümün önceki hâli |

**Bu tablo geçen sezonun hikâyesi.** Level 0'dan 5'e kadar altı ayrı tam sistem
yazılmış. Her seviye bir öncekinin daha yeteneklisi — ama sıfırdan değil, kopyalanarak.

*Gall's Law açısından ilginç olan:* aslında doğru yapılmış (basitten karmaşığa),
ama **eski seviyeler silinmediği için** depo altı kat şişmiş. Level 5 çalıştığında
0-4 çöp olmuştu, yine de Nisan'a kadar kodda durdular.

---

## Bu sezon için öneri

1. **Tek teleop OpMode, alliance parametreli.** `AllianceTeleopBase` fikrini geri getir.

2. **`RecoveryController`'ı ilk günden koy.** 195 satır, maç kurtarır.

3. **`ZoneController`'ı şablon al** — controller'lar böyle olmalı: tek sorumluluk,
   ~100 satır, dairesel bağımlılık yok.

4. **Controller katmanını erken kurma.** Mantık teleop içinde başlasın; **aynı kod
   üçüncü kez tekrarlandığında** sınıfa çıksın. Geçen sezon 5 controller / 2.000 satır
   ve birbirine setter'la bağlıydı.

5. **Dairesel bağımlılık yasak.** A → B ve B → A gerekiyorsa, üçüncü bir yerde
   birleştirilmeli.

6. **Kapatılan özellik silinir.** `if (false &&` yerine `git revert`.
   Bu, geçen sezonun en büyük tek dersi.

7. **Sürücü ergonomisini taşı:** rumble uyarıları, zone LED'i, 2 sn basılı tutma
   şartları, elle offset d-pad'leri, `FINISHING_PULSE` gibi yarıda kesmeyen davranışlar.
   Bunlar sahada öğrenilmiş şeyler, bedava geliyor.

8. **Tuning menüsünü Dashboard'a bırak**, sadece zorunlu 2-3 parametre için mini menü.

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
cat $P/contingency/lvbelc5/controllers/RecoveryController.java   # korundu
cat $P/contingency/lvbelc5/controllers/ZoneController.java       # korundu
cat $P/contingency/lvbelc5/teleop/BlueTeleop.java                # korundu
git show d7711d0^:$P/teleop/AllianceTeleopBase.java
```
