# 02 — Taret: 5 Varyant, Hall Kalibrasyon, Tarama Modu

## Özet

Geçen sezon taret, **en çok yeniden yazılan** subsystem'di: 5 farklı implementasyon
denendi, maça 5.'si girdi. Bu, subsystem interface'lerinin neden değerli olduğunun
en net kanıtı — 4 varyant çöpe gitti ama interface sayesinde denemek ucuzdu.

Taret donanımı alışılmadıktı: **CRServo ile sürülüyor, iki farklı encoder'dan
(analog mutlak + motor inkremental) Kalman füzyonu ile açı okunuyordu.**

---

## [A] MAÇTA KOŞTU

### `hardware/subsystems/turret/TurretPidPazarSubsystem.java` — 465 satır
**Korundu, hâlâ yerinde.** ("Pazar" = pazar günü yazıldığı için.)

Donanım:
- 2× `CRServo` (primary + secondary, aynı komut)
- 1× `DcMotorEx` sadece **encoder okumak için** (inkremental)
- 1× `AnalogInput` — mutlak pozisyon (potansiyometre/analog encoder)

Kontrol: tek PID (FTCLib `PIDController`) + **Kalman füzyonu**.

#### Kalman füzyonunun mantığı (146-204. satırlar)
```
PREDICTION : inkremental encoder delta'sı ile açıyı ilerlet
UPDATE     : analog encoder ölçümüyle düzelt
ENCODER ONLY: analog güvenilmezse sadece encoder
```
İlginç kısım **güven zamanlaması**: `fullTrustSec` süresince analog'a tam güvenilir,
sonra `fadeOutSec` boyunca güven azalır, sonrası sadece encoder. Yani analog encoder
başlangıç referansı için kullanılıyor, sürüşte encoder'a geçiliyor. Analog'un gürültülü
ama mutlak, encoder'ın temiz ama göreli olması problemine pratik çözüm.

Ek olarak analog sinyale **alçak geçiren filtre** (`analogLpfValue`) uygulanıyor.

#### `snapToZero` mekanizması
Init'te analog okuma sıfıra yakınsa açı tam sıfıra çekiliyor (`didSnapToZero`).
Kalibrasyon başlangıç hatasını önlüyor.

#### Kalibrasyon API'si
```java
resetCalibration()
resetCalibrationWithDuration(fullTrustSec, fadeOutSec)
finishCalibration()
isCalibrating() / getCalibrationElapsedSeconds() / getTotalCalibrationSec()
```
`Robot.java` init fazında `runCalibration()` ile çağırıyor, minimum 3 saniye
(`MIN_CALIBRATION_SEC`). Teleop başlamadan taret referansını buluyor.

**Bu sezon:** Taret donanımı değişirse bu kod da değişir. Ama **iki encoder füzyonu +
zamanla azalan güven** fikri saklanmaya değer; mutlak+göreli sensör birleştirme
problemi tekrar çıkar.

### `TurretInterface.java` — 61 satır
```java
setTargetAngleDegrees(double)   disableHold()       setManualPower(double)
getCurrentAngleDegrees()        zeroCurrentPosition()
setCurrentAngleOffset(double)   isAtTargetAngle()   isCalibrating()
```
Temiz arayüz. 5 implementasyonun ortak paydası olarak evrilmiş — yani gerçekten
kullanılmış bir soyutlama, spekülatif değil.

### `contingency/lvbelc5/controllers/AimingController.java` — 207 satır
Taret + hood + shooter'ı birlikte hedefe yönlendiriyor. `RonaldoShEngine`'den çözüm
alıp dağıtıyor.

Dikkat çeken: **fallback çözümü**
```java
FALLBACK_MIN_HOOD = 36.0   FALLBACK_MIN_RPM = 4200
FALLBACK_MAX_HOOD = 52.0   FALLBACK_MAX_RPM = 5850
```
Lokalizasyon güvenilmezse sabit değerlerle atış. `isUsingFallback()` ile telemetriye
veriliyor. Pratik ve doğru bir emniyet valfi — **bu fikri taşı.**

Ayrıca elle offset: `setTurretAngleOffset()` / `setHoodAngleOffset()` — sürücü
gamepad d-pad ile canlı düzeltme yapabiliyordu. Yarışmada kesin lazım olur.

`isTurretAtLimit()` → taret hedefe dönemiyorsa gamepad rumble ile uyarı.

---

## [C] TEST EDİLMEDİ — 4 eski varyant

Hepsi `d7711d0` ile silindi. Sırayla:

| Dosya | Satır | Yaklaşım |
|---|---|---|
| `TurretPidSubsystem.java` | 151 | En basit: tek PID, tek encoder |
| `CascadeTurretSubsystem.java` | 266 | Kaskad: pozisyon PID → hız PID |
| `TurretCascadeHallSubsystem.java` | 473 | Kaskad + Hall efekt sensörü ile sıfırlama |
| `TurretCascadeKalmanSubsystem.java` | 577 | Kaskad + Kalman füzyonu |
| `TurretPidKalmanSubsystem.java` | 458 | Tek PID + Kalman (→ Pazar'ın atası) |

**Evrim çizgisi net:** Pid → Cascade → Cascade+Hall → Cascade+Kalman → Pid+Kalman → **PidPazar**.
Kaskad denendi, bırakıldı; sonunda basit PID + iyi sensör füzyonu kazandı.
*Bu, Gall's Law'un olumlu örneği: karmaşık olan değil, basit olan maça girdi.*

### `HallCalibrator.java` — 292 satır [C]
Hall efekt sensörüyle taret sıfır noktası bulma. Hall'lı varyant terk edilince kullanılmadı.

### Taret komutları — `hardware/subsystems/turret/commands/` [C]
| Dosya | Satır |
|---|---|
| `TurretSetFieldOrientedCommand.java` | 98 |
| `TurretSetAngleCommand.java` | 82 |
| `TurretHomingCommand.java` | 58 |
| `TurretZeroCommand.java` | 38 |
| `TurretDisableCommand.java` | 15 |

FTCLib command tabanlı. Command deseni sonradan terk edilmiş (LC5 doğrudan
`subsystem.setTargetAngleDegrees()` çağırıyor).

### Taret tuning OpMode'ları — `tunaing/opmodes/turret/` [C] — 4.483 satır
| Dosya | Satır |
|---|---|
| `TurretCascadeAutoTune.java` | 1033 |
| `TurretAutoPazarRoutine.java` | 937 |
| `TurretAutoTuneRoutine.java` | 808 |
| `TurretCascadeParamTest.java` | 788 |
| `manual/TurretResponseProfiler.java` | 480 |
| `TurretVelocityTuner.java` | 338 |
| `TurretFFTuner.java` | 336 |
| `TurretPositionTuner.java` | 323 |
| `TurretPazarManualTest.java` | 238 |

**Kanıt durumu:** Shooter'ın aksine **hiçbir turret tuning çıktı CSV'si bulunamadı.**
`shooter_auto_runs` gibi bir `turret_auto_runs` yok. Diskte `*turret*.csv` veya
`*pazar*` veri dosyası yok.

→ Bu 4.483 satır muhtemelen hiç tam koşmadı. **Taşıma.** Gerekirse `FFActuator`
arayüzünü (`03-tuning-framework.md`) implement edip yeniden yaz — çatı zaten bunun için var,
`TurretFFActuator.java` (159 satır) o implementasyonun kendisi ve küçük.

### Taret ayar depoları — `settings/storage/turret/` [C] — 335 satır
`TurretCascadeParameterStorage` (138), `TurretPidInterpolationStorage` (79),
`TurretResponseStorage` (42), `TurretPazarStorage` (39), `TurretParameterStorage` (37).

Terk edilen varyantların parametreleri. `TurretPazarStorage` hariç hepsi ölü.

---

## Tarama modu (top arama) [C]

### `logic/advanced/commands/turret/ScanningTurretIdle.java` — 44 satır
```java
public IdleTurretOutput compute(RobotState ctx) {
    double t = ctx.timestampMs / 1000.0;
    double angle = Math.sin(t * frequency * 2 * Math.PI) * amplitude;
    return IdleTurretOutput.scanning(angle, ...);
}
```
Varsayılan: **±45°, 0.5 Hz sinüs taraması.** Boşta kalan tareti sağa sola süpürüp
vision ile top arıyor.

44 satır. **Kopyalamak yerine yeniden yaz** — bu kadarcık kod için git arkeolojisi yapmaya değmez.

### `TargetTrackingTurretIdle.java` — 61 satır [C]
Tarama yerine bilinen hedefi takip etme modu.

### `TurretIdleCommand.java` (48) + `TurretIdleCommandFactory.java` (52) [C]
Boşta davranış seçimi için arayüz + selector.

### İlgili: `ScanCoverageMap.java` — 383 satır [C]
**Bu daha değerli fikir.** Taretin nereyi taradığını, hangi bölgenin kör kaldığını
tutan kapsama haritası. Kör tarama yerine "en uzun süredir bakılmayan yöne dön"
stratejisi kurulabilir. Detay → `05-vision-ball-tracking.md`

---

## Bu sezon için öneri

1. **`TurretInterface`'i koru.** 5 varyat geçmişi bunun haklılığını kanıtlıyor.
   Bu sezon taret varsa yine birden fazla kontrol yaklaşımı denenecek.
2. **Basit PID ile başla.** Geçen sezonun evrimi kaskadın gereksiz olduğunu gösterdi.
3. **Sensör füzyonunu erteleme ama karmaşıklaştırma.** Mutlak+göreli encoder varsa
   `TurretPidPazarSubsystem`'in "zamanla azalan analog güveni" yaklaşımını taşı —
   çalıştığı kanıtlı.
4. **Fallback çözümünü ilk günden koy** (`AimingController`'daki sabit hood/RPM).
   Lokalizasyon bozulduğunda maç bitmez.
5. **Elle offset'i unutma.** D-pad ile canlı taret/hood düzeltmesi yarışmada hayat kurtarıyor.
6. **Tarama modunu 44 satır olarak yaz**, `ScanCoverageMap`'i sonraya bırak.
7. **Turret tuner'ı taşıma**, `FFActuator` implement et.
