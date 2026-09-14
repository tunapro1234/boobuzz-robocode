# 01 — Shooter: Subsystem, PIDF, Atış Motorları, Balistik

## Özet

Geçen sezon shooter, kod tabanının **en olgun** parçasıydı. Hem maçta koştu hem de
gerçek robot üzerinde otomatik tuning yapıldı ve çıktı verisi diskte duruyor.
Bu sezon devralınacak en güvenli kod burası.

---

## [A] MAÇTA KOŞTU

### `hardware/subsystems/shooter/ShooterPidfPowerSubsystem.java` — 311 satır
**Korundu (silinmedi), şu an `de-cock/robot-code`'da duruyor.**

Saf power-tabanlı hız PIDF kontrolcüsü. Tüm kazançlar power biriminde (0-1), voltaj
telafisi yok. Yapısı:

- **Integral zone + clamp**: hata `integralZone` dışındaysa integral sıfırlanır,
  içerideyse `integralMaxAccum` ile sınırlanır. Windup koruması doğru yapılmış.
- **Stability timer**: hata toleransta *kesintisiz* `stabilityDurationMs` kadar
  kalırsa `stable=true`. Tek örnekleme ile "hazır" demiyor — bu önemli, atış
  tetiklemesi buna bağlıydı.
- **Feedforward ayrımı**: `power = kS*sign + kV*rpm`, PID'den ayrı hesaplanıp
  toplanıyor. `lastFfPower` / `lastPidPower` ayrı ayrı telemetriye veriliyor —
  tuning sırasında hangisinin ne kadar katkı verdiği görülüyor.
- **Slew rate limiter**: `slewRatePowerPerSec` > 0 ise power değişimi sınırlanır.
  Geçen sezon `0.0` (kapalı) bırakılmış.
- **İki motorlu takip**: `encoderSource` (LEFT/RIGHT) seçili motordan hız okunur,
  diğerine `followerScale` ile power verilir.
- **1500 RPM eşiği**: `computeFeedforwardPower()` içinde `absTarget < 1500` ise
  FF sıfır döner. Düşük hızlarda FF modelinin geçersiz olduğu tespit edilmiş.

```bash
# Dosya hâlâ yerinde:
cat de-cock/robot-code/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/subsystems/shooter/ShooterPidfPowerSubsystem.java
```

**Bu sezon:** Neredeyse olduğu gibi taşınabilir. Oyun ne olursa olsun flywheel varsa
bu mantık geçerli. Tek değişiklik: `HardwareConstants.Shooter` bağımlılığı sadeleştirilmeli.

### Kalibre edilmiş kazançlar — `settings/storage/shooter/ShooterPidfPowerStorage.java`
```
kS = 0.18766200
kV = 0.00013514
kP = 0.00030984
kI = 0.00189683
kD = 1.26816e-05
integralZoneRpm   = 250.0
integralMaxAccum  = 3000.0
targetToleranceRpm = 100.0
stabilityDurationMs = 150
```
Bu sayılar elle yazılmadı — `tunaing` otomatik tuner'ının çıktısı (aşağıya bak).
**Yeni robotta geçersiz olacaklar**, ama tuner'ı tekrar koşturunca aynı formatta üretilir.

### `engines/RonaldoShEngine.java` — 704 satır
Mesafe → (shooter RPM, hood açısı) çözücü. Lookup tablosu + interpolasyon.

**Uyarı:** 704 satırın ~500'ü ölü alternatif. İki enum var:
- `ShooterRpmMethod`: LOOKUP_TABLE, LINEAR, LINEAR_INTERPOLATION, POLY_DEG2
- `HoodMethod`: LINEAR_INDEPENDENT, LINEAR_INTERPOLATION, POLY_DEG2_INDEPENDENT

Ama seçim `static final` hardcoded:
```java
private static final ShooterRpmMethod SHOOTER_RPM_METHOD = ShooterRpmMethod.LINEAR;
private static final HoodMethod HOOD_METHOD = HoodMethod.POLY_DEG2_INDEPENDENT;
```
Yani 7 yöntemden 2'si kullanılmış. `LINEAR` için yorumda `R² = 0.9656` yazıyor —
demek ki gerçek veriye fit edilmiş.

**Özü:** mesafe → tablo → (rpm, hood). Bu ~80 satır. **Bu sezon 80 satır olarak yaz.**

### `ShotSolution4.java` — 47 satır
Immutable sonuç nesnesi: `hoodAngleDeg`, `shooterRpm`, `isValid`, `distanceInch`.
`valid()` / `invalid()` static factory'leri var. Temiz, küçük, taşınır.

### `hood/HoodSubsystem.java` — 73 satır
Servo tabanlı hood açı kontrolü. Basit, sorunsuz.

---

## [B] ROBOTTA KOŞTU (tuning, maç değil)

### Kanıt
```
de-cock/shooter_auto_runs/
  lion_power/run_20251128_074912/
    pid_relay_2000.csv  pid_relay_2600.csv  pid_relay_3200.csv
    pid_relay_3800.csv  pid_relay_4200.csv  pid_relay_4500.csv
    pid_relay_4800.csv  pid_relay_5000.csv
    ff_big.csv  ff_small.csv
  lion_power/shooter_response_detailed_20251128_075450.csv
  lion_vcomp/run_20251128_084506/   (aynı set)
  lion_vcomp/shooter_response_detailed_20251128_085102.csv
```
Dosya adları `PIDRelayRunner` (prefix `pid_relay`) ve `FFRunner`'ın çıktı formatıyla
birebir eşleşiyor. **28 Kasım 2025'te gerçek robotta iki ayrı tuning oturumu yapılmış**
("lion" = o tarihteki robot/batarya adı; `power` ve `vcomp` iki farklı shooter varyantı).

### İlgili tuner OpMode'ları — hepsi [B]
| Dosya | Satır | İş |
|---|---|---|
| `tunaing/opmodes/shooter/auto/ShooterPowerAutoTune.java` | 561 | Power-tabanlı otomatik PID+FF tuning |
| `tunaing/opmodes/shooter/auto/ShooterVcompAutoTune.java` | 550 | Voltaj-telafili varyant için aynısı |
| `tunaing/opmodes/shooter/manual/ShooterResponseProfiler.java` | 507 | Basamak yanıtı profilleme → CSV |
| `tunaing/opmodes/shooter/ShooterPIDTuner.java` | 341 | Elle PID ayarı |
| `tunaing/opmodes/shooter/ShooterFFTuner.java` | 325 | Elle FF ayarı |
| `tunaing/opmodes/shooter/ShooterRangeTest.java` | 170 | Menzil taraması |
| `tunaing/profiler/ShooterProfiler.java` | 232 | Profilleme yardımcısı |

Çatı detayları → `03-tuning-framework.md`

### Analiz tarafı — `de-cock/shooter_auto_runs/*.py`
| Dosya | Satır | İş |
|---|---|---|
| `battery_analysis.py` | 36K | Batarya voltajı → RPM sapması analizi |
| `estimator_analysis.py` | 30K | Farklı hız tahmincilerinin karşılaştırması |
| `analyze_shooter.py` | 19K | Ana analiz |
| `lowpass_analysis.py` | 16K | Encoder gürültüsü için alçak geçiren filtre |
| `test_improved_estimators.py` | 16K | Geliştirilmiş tahminciler testi |
| `battery_estimators.py` | 12K | Batarya modeli |
| `improved_estimators.py` | 11K | Tahminci implementasyonları |

Çıktı grafikleri `analysis_output/` içinde (step response, error distribution,
power command analysis, time series overlay).

**Bu sezon:** Bu Python analiz zinciri değerli. Encoder gürültüsü ve batarya düşüşü
her sezon aynı problem. Aynı CSV formatını üretirsek scriptler doğrudan çalışır.

---

## [C] TEST EDİLMEDİ

### Atış motoru ailesi — `logic/advanced/engines/shot/`
| Dosya | Satır | Not |
|---|---|---|
| `ShotEngine.java` | 37 | Arayüz |
| `ShotSolution.java` | 70 | Sonuç nesnesi (LC5'teki `ShotSolution4`'ün atası) |
| `ShotEngineFactory.java` | 41 | Selector |
| `CalibrationLookup.java` | 217 | Tablo interpolasyonu |
| `RonaldinhoEngine.java` | 221 | Fizik tabanlı çözücü |
| `RonaldoEngine.java` | 164 | Lookup tabanlı (LC5'teki `RonaldoShEngine`'in atası) |
| `HaalandEngine.java` | 62 | ? |
| `MessiEngine.java` | 34 | `ball-auto-istic/messi` motorunun robot tarafı |

Futbolcu isimleri = farklı balistik yaklaşımlar. Sadece **Ronaldo** kolu maça kadar
gelmiş (`RonaldoShEngine` olarak), diğerleri yolda kalmış.

`MessiEngine` 34 satır — muhtemelen `ball-auto-istic`'in ürettiği Java tablosunu
okuyan ince bir sarmalayıcı.

### `config/ShooterCalibration.java` — silinmiş, satır sayısı için envanter dosyasına bak

---

## Harici repo: `de-cock/ball-auto-istic` (120 MB, 34 commit)

Atış fiziği laboratuvarı. Branch `stable`, son commit "rk4 optimization" (7 Ara 2025).

```
messi/          — çözücü motoru + kalibrasyon (shooter_calibration.json)
ronaldinho/     — ikinci motor: calibration_2d.py, engine_mirror.py, physics.py,
                  robustness.py, trajectory_scoring.py, field_solution.py
physics/        — bullet_sim.py (PyBullet), engine.py, models.py, variation.py
solver/         — Java çıktıları: TargetRpmCalculator.java, ShooterCalibrator.java,
                  RpmVelocityCheck.java, ShooterLogger.java
calibration/    — 4 adımlı iş akışı: step1_capture → step2_trajectory →
                  step3_analysis → step4_physics (CALIBRATION_WORKFLOW.md)
tools/          — calibrate_shooter.py, min_rpm_solver.py, sim_3d.py, gui.py,
                  verify_bounce.py, verify_physics.py, verify_solver.py
```

**Kritik nokta:** Bu repo **Java kodu üretiyor** (`solver/*.java`, `messi/output/*.java`).
Yani Python'da fizik çöz → Java lookup tablosu üret → robota göm. Bu iş akışı
bu sezon da geçerli.

`calibration/` altındaki 4 adımlı akış (video yakala → yörünge çıkar → analiz → fizik fit)
top hızı ölçmenin pratik yolu. `CALIBRATION_WORKFLOW.md` oku.

**Commit dışı duran şeyler:** `frc/`, `messi/output/CoarseRK4.java`, `MessiINM.java`,
`MessiRK4NM.java`, `NelderMead.java`, `messi/output/yigit/`. Nelder-Mead optimizasyonu
ve RK4 entegratörü — muhtemelen en son çalışılan yer.

## Harici repo: `de-cock/shooter-heatmap` (24 MB, 1 commit)

`combined_rpm.csv` ve `shooter_heatmap_rpm.csv` — her biri 9.3 MB, format `x_in,y_in,rpm`.
Sahanın her noktası için gereken RPM. `generate_heatmap.py` (13K) bunu PNG'ye çeviriyor
(`shooter_heatmap_empty.png`, `shooter_heatmap_loaded.png` — boş ve yüklü robot).

**Bu sezon:** Aynı fikir birebir tekrarlanabilir. Saha üstünde nereden atış yapılabilir
sorusunun görsel cevabı, strateji toplantısında işe yarar.

---

## Bu sezon için öneri

1. **`ShooterPidfPowerSubsystem`'i taşı**, `HardwareConstants` bağımlılığını sadeleştir.
2. **`ShooterInterface`'i koru** — geçen sezon 2 implementasyonu oldu (power, vcomp),
   bu sezon da varyant denenecek.
3. **Atış çözücüsünü 80 satır yaz.** `RonaldoShEngine`'in 7 yöntemini taşıma; `LINEAR`
   RPM + `POLY_DEG2` hood zaten kazanan kombinasyondu, onu doğrudan yaz.
4. **Tuner'ı ilk gün kur** (`03-tuning-framework.md`). Geçen sezon 28 Kasım'da tune
   edildi — sezon başlayalı 2 ay olmuştu. Bu sefer erken.
5. **`ball-auto-istic` iş akışını koru**, ama yeni oyunun mermisine göre fizik modelini
   yeniden fit et. Kod değil, *süreç* devrediliyor.
