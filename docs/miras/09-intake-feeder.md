# 09 — Intake ve Feeder: Top Alma, Besleme, Sayma

## Özet

Küçük ama kritik iki subsystem. Maça giren hâlleri çok sade (42 ve 176 satır) —
**bu bir başarı işareti, eksiklik değil.** Karmaşık varyantlar (PIDF intake,
Cascade feeder) denenip bırakılmış.

En değerli miras: **feeder'ın "pulse" (darbe) modeli** ve **intake'teki top sayma**.

---

## [A] MAÇTA KOŞTU

### `hardware/subsystems/intake/IntakePowerSubsystem.java` — 42 satır
Olabilecek en sade hâl:
```java
motor.setMode(RUN_WITHOUT_ENCODER);
motor.setZeroPowerBehavior(BRAKE);
runDefault()  → motor.setPower(HardwareConstants.IntakePower.defaultPower);  // 1.0
run(power)    → motor.setPower(power);
stop()        → motor.setPower(0.0);
```
Encoder yok, PID yok, sensör yok. **42 satır ve maça giren bu.**

Üç varyant yazıldı, en basiti kazandı. (`POWER_SENSOR` seçiliydi `SubsystemSelection`'da
ama `Robot.java` doğrudan `IntakePowerSubsystem` kuruyor — sensörlü sürüm silinmiş.)

### `hardware/subsystems/intake/IntakeInterface.java` — 31 satır
```java
void runDefault();
void stop();
default void setTargetRpm(double)        { }
default double getTargetRpm()            { return 0; }
default double getCurrentRpm()           { return 0; }
default boolean isAtTargetVelocity()     { return false; }
default boolean isIntakeFull()           { return false; }   // top algılama
```
**`default` metot kullanımı doğru:** basit varyant hiçbirini implement etmek zorunda
değil, gelişmiş varyant ezer. Interface'in maliyetini sıfıra indiriyor.

### Top sayma altyapısı — `HardwareConstants.Intake`
```java
String  motorName = "intake";
boolean reverseDirection = true;
double  encoderTicksPerRev = 28.0;
double  gearRatio = 1.0;

String  ballSensorName = "intake_dist";   // mesafe sensörü
int     maxBallCapacity = 3;
double  distanceEmptyInch = 6.0;          // bu mesafe = boş
double  distanceBallInch  = 3.0;          // bu mesafe = top var
double  holdPower = 0.2;                  // topu tutma gücü
```
**Eşik tabanlı basit top algılama.** Kamera değil, tek mesafe sensörü. 3 top kapasitesi.
`holdPower = 0.2` — topları içeride tutmak için hafif güç.

**Bu sezon toplar dağınık olacak** (Tuna'nın notu) → kaç top taşındığını bilmek
daha da önemli olacak. Bu eşik yaklaşımı ucuz ve çalışıyor; kameradan önce bunu kur.

### `hardware/subsystems/feeder/FeederPowerSubsystem.java` — 176 satır
### `hardware/subsystems/feeder/FeederInterface.java` — 138 satır ⭐ *pulse modeli değerli*

Feeder'ın işi: shooter hazır olduğunda **tek top** göndermek. Arayüz bunu
"pulse" (darbe) soyutlamasıyla çözmüş:

```java
// Temel kontrol
void setPower(double)            // default {} — Cascade varyantı yoksayar
void stop();
boolean isRunning();
default boolean isPulseComplete() { return !isRunning(); }

// Darbe yönetimi
default void requestPulse()      { }   // darbe iste; sürüyorsa bitince yenisi başlar
default void clearRequest()      { }   // isteği iptal et; mevcut darbe biter
default boolean isPulsing()      { return false; }

// Gecikmeli darbe
requestPulseAndDelay(long delayMs)     // darbe → bekle → darbe (istek sürdükçe)
```

**Neden bu model doğru:**
- `requestPulse()` **kuyruk mantığı** — darbe ortasında yeni istek gelirse kaybolmuyor,
  sıraya giriyor. Sürücü tetiğe basılı tutunca toplar teker teker gidiyor.
- `clearRequest()` + "mevcut darbe biter" → **yarım kalan atış yok**.
  `ShootingController`'ın `FINISHING_PULSE` state'i buna dayanıyor
  (`07-teleop-controllers.md`).
- `isPulsing()` dışarıdan sorgulanabiliyor — shooter RPM'de kalmaya devam etsin mi
  kararı buna bakıyor.
- `requestPulseAndDelay(FEEDER_DELAY_MS = 100)` → ardışık atışlar arası 100 ms.

**Bu sezon: pulse modelini olduğu gibi taşı.** Mekanizma ne olursa olsun
"tek birim gönder, kuyruklu, yarıda kesilmez" davranışı aynı.

### Jam (sıkışma) temizleme — `ShootingController` içinde
İki ayrı jam kurtarma var:
```java
executeJamClear()          // LB → intake + feeder ters yön
executeShooterJamClear()   // Y  → shooter ileri/geri salınım, 500 ms periyot
    SHOOTER_JAM_TOGGLE_MS = 500
```
**Sahada öğrenilmiş şeyler.** Top sıkışması FTC'de kaçınılmaz; sürücünün tek tuşla
kurtarabilmesi maç kazandırır. **Bu sezon ilk günden koy.**

---

## [C] TEST EDİLMEDİ

### Intake varyantları
| Dosya | Satır | Ne |
|---|---|---|
| `IntakePidfSubsystem.java` | 157 | Hız kontrollü (velocity PIDF) |
| `IntakePowerSensorSubsystem.java` | 134 | Power + mesafe sensörüyle top algılama |
| `IntakeFactory.java` | 27 | Selector |
| `commands/IntakeRunCommand.java` | 33 | FTCLib command |
| `commands/IntakeStopCommand.java` | 28 | FTCLib command |

`IntakePowerSensorSubsystem` ilginç — `SubsystemSelection.intakeType = POWER_SENSOR`
varsayılanıydı ama `Robot.java` onu değil sade `IntakePowerSubsystem`'i kuruyor.
Yani seçim ayarı ile gerçek kod uyuşmuyordu.

`HardwareConstants.IntakePIDF` (kS, kV, kP, kI, kD, integralZone, targetTolerance,
stabilityDuration) tanımlı ama kullanılmamış.

### Feeder varyantı
| Dosya | Satır | Ne |
|---|---|---|
| `FeederCascadeSubsystem.java` | 414 | Hız + pozisyon kaskad PID (taret gibi) |
| `FeederFactory.java` | 25 | Selector |
| `settings/storage/feeder/FeederCascadeStorage.java` | 68 | Kaskad parametreleri |

414 satırlık kaskad feeder yazılmış, 176 satırlık basit sürüm kazanmış.
*Taretle aynı hikâye (`02-turret.md`): kaskad denendi, basit kazandı.*

### Tuning OpMode'ları [C] — 1.183 satır
| Dosya | Satır |
|---|---|
| `tunaing/opmodes/feeder/FeederCascadePowerAutoTune.java` | 777 |
| `tunaing/opmodes/intake/IntakePidfPowerAutoTune.java` | 406 |
| `tunaing/ff/FeederFFActuator.java` | 84 |
| `tunaing/ff/IntakeFFActuator.java` | 84 |

Çıktı CSV'si diskte yok. Ama `FeederFFActuator` / `IntakeFFActuator` **84'er satır** —
tuning çatısına bağlanmanın ne kadar ucuz olduğunun kanıtı (`03-tuning-framework.md`).

### Testler [C]
`test/FeederJamTest.java`, `test/FeederShooterTest.java`,
`test/subsystems/phase1/Test02_Intake.java`, `Test03_Feeder.java`,
`test/subsystems/phase2/Test02_Intake.java`, `Test03_FeederCommand.java`,
`Test03_FeederManual.java`

**"phase1 / phase2" ayrımı ilginç:** phase1 = tek subsystem izole test,
phase2 = entegre test. Test OpMode'larını aşamalara bölmek iyi fikir, bu sezon tekrarla.

---

## Bu sezon için öneri

1. **Feeder pulse modelini taşı.** `requestPulse()` / `clearRequest()` / `isPulsing()` /
   `requestPulseAndDelay()` — 138 satırlık arayüzün özü bu dört metot. Mekanizma
   değişse de davranış aynı.

2. **Basit başla.** 42 satırlık `IntakePowerSubsystem` maça girdi; 157 satırlık PIDF
   sürümü girmedi. Hız kontrolü gerçekten gerekli olana kadar power yeter.

3. **Top sayma için mesafe sensörü eşiği kur** (`distanceEmptyInch` / `distanceBallInch`
   deseni). Toplar dağınık olacaksa kaç tane taşıdığını bilmek şart. Kameradan önce bu.

4. **Jam clear'ı ilk günden koy.** İki tuş: intake ters, shooter salınım.

5. **`default` metotlu interface deseni kullan** — basit implementasyon hiçbir şey
   yazmak zorunda kalmasın, gelişmiş olan ezsin. Interface maliyetini sıfırlıyor.

6. **phase1/phase2 test OpMode ayrımını tekrarla.**

7. **Kaskad kontrolü varsayılan yapma.** Hem taret hem feeder'da denendi, ikisinde de
   basit sürüm kazandı.

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
cat $P/hardware/subsystems/feeder/FeederInterface.java        # korundu ⭐
cat $P/hardware/subsystems/feeder/FeederPowerSubsystem.java   # korundu
cat $P/hardware/subsystems/intake/IntakeInterface.java        # korundu
cat $P/hardware/subsystems/intake/IntakePowerSubsystem.java   # korundu
grep -n -A25 "class Intake " $P/config/HardwareConstants.java # top sayma eşikleri
git show d7711d0^:$P/hardware/subsystems/intake/IntakePowerSensorSubsystem.java
git show d7711d0^:$P/hardware/subsystems/feeder/FeederCascadeSubsystem.java
```
