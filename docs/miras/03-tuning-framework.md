# 03 — `tunaing/`: Otomatik PID/FF Tuning Çatısı

## Özet

12.986 satırlık bir otomatik kalibrasyon çatısı. **Geçen sezonun en değerli ve en
kanıtlanmış mühendislik ürünü** — ama sadece shooter kolu için.

Mimari doğru kurulmuş: küçük bir soyutlama (`FFActuator`) + jenerik koşucular
(`FFRunner`, `PIDRelayRunner`) + mekanizma başına ince adaptör. Koşucuyu bir kez yaz,
her mekanizmaya tak.

**Bu sezon devralınacak çekirdek: ~850 satır.** Geri kalan 12.000 satır mekanizmaya
özel OpMode'lar ve büyük ölçüde test edilmemiş.

---

## Çekirdek çatı — taşınacak kısım

### `tunaing/ff/FFActuator.java` — 31 satır [B] ⭐
```java
public interface FFActuator {
    void configure();                    // encoder reset, mode ayarı
    double setCommand(double power);     // -1..1 power ver, uygulanan değeri dön
    double getVelocityRpm();
    double getBusVoltage();
    void stop();
    void waitForSpinDown(LinearOpMode opMode, long timeoutMs) throws InterruptedException;
}
```
**Çatının kalbi bu 6 metot.** Bir mekanizma bu arayüzü implement ederse, tüm otomatik
tuning makinesi ona takılıyor. `getBusVoltage()` olması önemli — batarya düşüşünü
ölçüme dahil ediyor.

### `tunaing/ff/FFRunner.java` — 449 satır [B] ⭐
Feedforward karakterizasyonu: farklı power seviyelerinde kararlı hâl hızını ölçüp
`power = kS + kV·rpm` doğrusunu çıkarır.

Akıllı kısımları:
- **Kararlılık tespiti**: kayan pencere (`windowMs`) üzerinde standart sapma *ve*
  eğim kontrolü. `stdPercent`/`stdMinRpm` ve `slopePercent`/`slopeMinRpm` eşikleri —
  hem gürültü hem trend bakılıyor. "Hız oturdu mu" sorusunun doğru cevabı.
- **Çok geçişli ölçüm**: `buildPasses()` power listesini birden çok geçişte, sırası
  değiştirilerek koşuyor. Histerezis ve ısınma etkisini yakalamak için.
- **Spin-down bekleme**: adımlar arası mekanizmanın durmasını bekliyor (`coastMsBetween`).
- **İki log seviyesi**: `ff_big.csv` (ham örnekler) + `ff_small.csv` (adım özetleri).

### `tunaing/ff/FFProfile.java` — 107 satır [B]
Builder deseniyle koşu parametreleri:
```
stepCount, minPower, maxPower, loopDelayMs, windowMs, holdMs,
stdPercent, stdMinRpm, slopePercent, slopeMinRpm, maxRpm,
stepTimeoutMs, coastMsBetween, spinDownTimeoutMs, enableBigLog, filePrefix
```
`FFProfile.shooterProfile()` hazır preset olarak duruyor.

### `tunaing/pid/PIDRelayRunner.java` — 169 satır [B] ⭐
**Relay (röle) yöntemiyle otomatik PID** — Åström–Hägglund. Mekanizmayı iki power
seviyesi arasında (`highPower`/`lowPower`) hedef RPM etrafında salındırır, oluşan
limit çevriminin **periyodunu ve genliğini** ölçer. Buradan ultimate gain/period ve
Ziegler-Nichols türevi kazançlar çıkar.

```java
public static class Result {
    public final File csvFile;
    public final double averagePeriod;   // limit çevrimi periyodu
    public final double amplitude;       // salınım genliği
    public final double maxRpm, minRpm, averageRpm;
}
```
Elle PID çevirmekten **çok daha hızlı ve tekrarlanabilir**. Canlı FTC Dashboard
telemetrisi de var.

### `tunaing/pid/PIDRelayProfile.java` — 94 satır [B]
```java
PIDRelayProfile.shooterProfile()
    .highPower(1.0).lowPower(0.55)
    .hysteresisRpm(40.0).testDurationSec(10.0)
    .loopDelayMs(20).enableCsvLog(true)
    .filePrefix("tunapro_pid_relay")
```

### Mekanizma adaptörleri — `tunaing/ff/`
| Dosya | Satır | Durum |
|---|---|---|
| `ShooterFFActuator.java` | 109 | **[B]** koştu |
| `TurretFFActuator.java` | 159 | [C] |
| `IntakeFFActuator.java` | 84 | [C] |
| `FeederFFActuator.java` | 84 | [C] |

Adaptörlerin küçüklüğüne dikkat: **~100 satır.** Yeni bir mekanizmayı tuning çatısına
bağlamanın maliyeti bu kadar. Çatının değeri burada.

---

## [B] KANIT: gerçekten koştu

```
de-cock/shooter_auto_runs/
  lion_power/run_20251128_074912/
    ff_big.csv  ff_small.csv
    pid_relay_2000.csv  pid_relay_2600.csv  pid_relay_3200.csv  pid_relay_3800.csv
    pid_relay_4200.csv  pid_relay_4500.csv  pid_relay_4800.csv  pid_relay_5000.csv
  lion_power/shooter_response_detailed_20251128_075450.csv
  lion_vcomp/run_20251128_084506/   ← aynı set, vcomp varyantı için
  lion_vcomp/shooter_response_detailed_20251128_085102.csv
```

**28 Kasım 2025, iki oturum.** `pid_relay_<rpm>.csv` adları relay koşusunun 8 farklı
hedef RPM'de tekrarlandığını gösteriyor — yani kazançlar tek noktada değil, menzil
boyunca çıkarılmış.

Sonuç: `ShooterPidfPowerStorage`'daki `kS=0.18766200, kV=0.00013514, kP=0.00030984,
kI=0.00189683, kD=1.26816e-05`. Bu basamak sayıları elle yazılmaz.

---

## [B] Shooter tuning OpMode'ları — 2.436 satır
| Dosya | Satır |
|---|---|
| `opmodes/shooter/auto/ShooterPowerAutoTune.java` | 561 |
| `opmodes/shooter/auto/ShooterVcompAutoTune.java` | 550 |
| `opmodes/shooter/manual/ShooterResponseProfiler.java` | 507 |
| `opmodes/shooter/ShooterPIDTuner.java` | 341 |
| `opmodes/shooter/ShooterFFTuner.java` | 325 |
| `opmodes/shooter/ShooterRangeTest.java` | 170 |
| `profiler/ShooterProfiler.java` | 232 |

`*AutoTune` OpMode'ları FF + relay PID koşusunu uçtan uca otomatikleştiriyor: robota
tak, OpMode'u başlat, CSV'lerle dön.

---

## [C] TEST EDİLMEDİ — 8.500+ satır

### Taret kolu — 4.483 satır
`TurretCascadeAutoTune` (1033), `TurretAutoPazarRoutine` (937), `TurretAutoTuneRoutine` (808),
`TurretCascadeParamTest` (788), `manual/TurretResponseProfiler` (480), `TurretVelocityTuner` (338),
`TurretFFTuner` (336), `TurretPositionTuner` (323), `TurretPazarManualTest` (238).

**Hiçbir turret CSV çıktısı diskte yok.** Taşıma.

### Feeder / intake kolu — 1.183 satır
`FeederCascadePowerAutoTune` (777), `IntakePidfPowerAutoTune` (406). Çıktı verisi yok.

### Vision kolu — 2.415 satır
`LimelightCalibrator` (517), `HuskyLensAutoTune` (438), `LimelightCalibratedTest` (417),
`LimelightKalmanTest` (413), `LimelightMT1MT2Test` (311), `LimelightAprilTagTest` (227),
`HuskyLensColorTest` (92). Detay → `04-localization.md` ve `05-vision-ball-tracking.md`.

### `profiler/TurretProfiler.java` — 135 satır [C]

---

## Python analiz zinciri — `de-cock/shooter_auto_runs/`

CSV'leri işleyen tarafı. Bu scriptler gerçek veriyle çalıştı:

| Dosya | Boyut | İş |
|---|---|---|
| `battery_analysis.py` | 36K | Batarya voltajı → RPM sapması |
| `estimator_analysis.py` | 30K | Hız tahmincisi karşılaştırma |
| `analyze_shooter.py` | 19K | Ana analiz + grafik |
| `lowpass_analysis.py` | 16K | Encoder gürültüsü filtreleme |
| `test_improved_estimators.py` | 16K | Tahminci testleri |
| `battery_estimators.py` | 12K | Batarya modeli |
| `improved_estimators.py` | 11K | Tahminci implementasyonları |

Çıktılar `analysis_output/`: `01_step_response_comparison.png`,
`02_error_distribution_comparison.png`, `03_power_command_analysis.png`,
`04_time_series_overlay.png`.

**Encoder gürültüsü ve batarya düşüşü her sezon aynı problem.** CSV formatını korursak
bu scriptler doğrudan çalışır.

---

## Bu sezon için öneri

1. **Çekirdeği ilk hafta taşı (~850 satır):**
   `FFActuator` + `FFRunner` + `FFProfile` + `PIDRelayRunner` + `PIDRelayProfile`.
   Bunlar mekanizmadan bağımsız, yeni oyunda da aynen geçerli.

2. **Adaptörleri ihtiyaç doğdukça yaz.** Her biri ~100 satır. `ShooterFFActuator`'ı
   şablon olarak al.

3. **OpMode'ları taşıma.** `ShooterPowerAutoTune`'un 561 satırı büyük ölçüde menü/telemetri.
   Çekirdek çatı elde olunca yeniden yazmak daha temiz.

4. **Tuning'i sezonun başına al.** Geçen sezon 28 Kasım'da tune edildi — ilk yarışma
   8-9 Kasım'daydı, yani *yarışmadan sonra*. Bu sefer robot döner dönmez.

5. **CSV formatını koru** ki Python analiz zinciri çalışsın.

6. **Turret/feeder/intake tuner'larını unut.** Çatı + adaptör yaklaşımı zaten onları
   ucuzlatıyor.

```bash
# Çekirdeği çıkarmak için:
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
for f in ff/FFActuator ff/FFRunner ff/FFProfile pid/PIDRelayRunner pid/PIDRelayProfile ff/ShooterFFActuator; do
  git show d7711d0^:$P/tunaing/$f.java > /tmp/$(basename $f).java
done
```
