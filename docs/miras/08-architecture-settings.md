# 08 — Mimari: Interface / Factory / Storage / Settings

> 🎯 **Tuna'nın açık talebi (14 Eyl 2026):**
> *"subsystemlar için interfaceleri istiyorum açıkçası farklı şeyler denemeyi rahatlatıyor."*
> *"selector mekanizmamız vardı interfaceler için seçim falan yapıyorduk."*
> *"interfacelerden select etme işini salmıştım, interfacelerden direkt kod içinden
> select ediyordum gibi hatırlıyorum."*
>
> Bu dosya o mekanizmanın tam dökümü.

---

## Interface'ler haklı çıktı — kanıt

İlk bakışta "her interface'in tek implementasyonu var" görünüyor (son temizlenmiş
hâlde doğru). **Ama geçmişe bakınca tablo tersine dönüyor:**

| Interface | Toplam implementasyon | Maça giren |
|---|---|---|
| `TurretInterface` | **5** (Pid, Cascade, CascadeHall, CascadeKalman, PidKalman → PidPazar) | PidPazar |
| `IntakeInterface` | **3** (Power, Pidf, PowerSensor) | PowerSensor→Power |
| `ShooterInterface` | **2** (PidfPower, PidfVcomp) | PidfPower |
| `FeederInterface` | **2** (Power, Cascade) | Power |
| `CartographerInterface` | 2 (Passthrough, PiriReis) | — |
| `LocalizerInterface` | 4 (Ekf, KalmanFusion, PedroPassthrough, Fusion) | — |
| `MotionController` | 2 (Passthrough, Ashtar) | — |
| `ShotEngine` | 4 (Ronaldo, Ronaldinho, Messi, Haaland) | Ronaldo |
| `StrategyEngine` | 1 (Fabius) | — |
| `VisionSource` | 2 (HuskyLens, Limelight) | Limelight |

**12 interface, 27 implementasyon.** Soyutlama spekülatif değildi, gerçekten kullanıldı.
Sonradan kazananlar kalıp gerisi silinince "tek implementasyon" illüzyonu oluştu.

---

## [A] Selector mekanizması — maça giren hâli

### `config/HardwareConstants.SubsystemSelection` (797-827. satırlar)
```java
public static final class SubsystemSelection {
    public enum TurretVariant  { PID, CASCADE, HALL, KALMAN }
    public enum ShooterVariant { PIDF, PIDF_VCOMP }
    public enum IntakeVariant  { POWER, PIDF, POWER_SENSOR }
    public enum FeederVariant  { POWER, CASCADE }

    public static TurretVariant  turretType  = TurretVariant.PID;
    public static ShooterVariant shooterType = ShooterVariant.PIDF;
    public static IntakeVariant  intakeType  = IntakeVariant.POWER_SENSOR;
    public static FeederVariant  feederType  = FeederVariant.POWER;
}
```
**Bu, "kod içinden seçme" mekanizmasının kendisi.** Statik alan, kodda düzenleniyor,
derleme zamanında sabit gibi davranıyor.

### Factory'ler — her biri 25-30 satır
```java
// hardware/subsystems/turret/TurretFactory.java  (29 satır)
public static TurretInterface create(HardwareMap hardwareMap) {
    switch (HardwareConstants.SubsystemSelection.turretType) {
        case CASCADE: return new CascadeTurretSubsystem(hardwareMap);
        case HALL:    return new TurretCascadeHallSubsystem(hardwareMap);
        case KALMAN:  return new TurretCascadeKalmanSubsystem(hardwareMap);
        case PID:
        default:      return new TurretPidSubsystem(hardwareMap);
    }
}
```
| Factory | Satır |
|---|---|
| `TurretFactory` | 29 |
| `ShooterFactory` | ~22 |
| `IntakeFactory` | 27 |
| `FeederFactory` | 25 |
| `LocalizerFactory` | 37 |
| `FusionLocalizerFactory` | 86 |
| `CartographerFactory` | 37 |
| `ShotEngineFactory` | 41 |
| `MotionControllerFactory` | 24 |
| `StrategyEngineFactory` | 48 |
| `TurretIdleCommandFactory` | 52 |
| `ShooterIdleCommandFactory` | 70 |
| `MotionCommandFactory` | 35 |

**Toplam ~530 satır, 13 factory.** Maliyeti düşük, faydası gerçek.

> 🚩 Ama maça giren LC5, factory'leri **kullanmıyor**:
> ```java
> // Robot.java
> shooter = new ShooterPidfPowerSubsystem(hardwareMap);   // doğrudan
> turret  = new TurretPidPazarSubsystem(hardwareMap);     // doğrudan
> ```
> Varyant denemesi bitince factory katmanı atlanmış. Mantıklı bir son hâl.

### [C] Çalışma zamanı seçimi denemesi — `settings/storage/settings/HardwareSettings.java` (50 satır)
```java
@Config   // FTC Dashboard'dan canlı değiştirilebilir
public final class HardwareSettings {
    public static ShooterVariant shooterType = SubsystemSelection.shooterType;
    public static TurretVariant  turretType  = SubsystemSelection.turretType;
    ...
    public static void apply() { SubsystemSelection.shooterType = shooterType; ... }
}
```
Dashboard'dan varyant değiştirip `apply()` ile geri yazma. **Bu, Tuna'nın "saldım"
dediği kısım** — `ShooterFactory` bunu okuyordu, `TurretFactory` doğrudan
`HardwareConstants`'a bakıyordu. Yani mekanizma yarıda kalmış, iki factory iki farklı
kaynaktan okuyor.

---

## [A] Storage deseni — kalibre değerlerin yeri

`settings/storage/` = tuning çıktısı olan sayıların tutulduğu yer, koddan ayrı.

**Maça giren (korunan):**
| Dosya | İş |
|---|---|
| `storage/PoseStorage.java` | Auto→teleop poz + taret açısı devri |
| `storage/shooter/ShooterPidfPowerStorage.java` | Kalibre PIDF kazançları |
| `storage/vision/LimelightCalibrationStorage.java` | 12 sütunlu vision kalibrasyon şeması |

**Silinen [C] — 928 satır:**
`turret/` 5 dosya (335), `settings/` 5 dosya (308), `feeder/FeederCascadeStorage` (68),
`intake/IntakePidfStorage` (46), `shooter/ShooterResponseStorage` (45),
`shooter/ShooterVcompStorage` (44), `vision/HuskyLensCalibrationStorage` (82).

Terk edilen varyantların parametreleri.

**Desen doğru:** kalibrasyon çıktısı `Storage`'da, donanım kimliği `Constants`'ta,
seçim `Selection`'da. Ama geçen sezon **dört katman** oluştu:
```
HardwareConstants  (isimler, oranlar, varsayılanlar)   828 satır
  ↓
SubsystemSelection (hangi varyant)
  ↓
Settings           (Dashboard'dan canlı değişim)        308 satır
  ↓
Storage            (kalibre değerler)                   928 satır
  ↓
Factory            (nesne üretimi)                      530 satır
```
**Bir subsystem eklemek 5 dosyaya dokunmak demekti.** Asıl karmaşıklık interface'te
değil, bu kuyrukta.

### `config/HardwareConstants.java` — 828 satır
Tek dosyada iç içe statik sınıflar: `Electrical`, `Chassis`, `Intake`, `IntakePower`,
`IntakePIDF`, `Shooter`, `ShooterPIDF`, `ShooterPIDFVcomp`, `ShooterResponse`,
`Turret`, `TurretPidPazar`, ... `SubsystemSelection`.

`Chassis` iyi bir örnek: sınır kutusu (mm) + türetilmiş birimler (cm, inç) + iç/dış
yarıçap. `HoodSubsystem` ve hard-reset bunları kullanıyor.

`Shooter` içinde birim dönüşüm yardımcıları: `motorRpmToWheel()`, `wheelRpmToMotor()`,
`wheelSurfaceSpeedInPerSec()`, `formatWheelMotorRpm()`. **Motor RPM ↔ tekerlek RPM
ayrımı önemli** (`motorToWheelRatio = 1.6`), karıştırılırsa her şey bozulur.

### `config/FieldConstants.java` + `config/Alliance.java`
`Alliance` enum (40 satır): `isRed()`, `isBlue()`, `getScorePosition()`,
`getHumanPlayerPosition()`, `fromBoolean()`. Temiz, taşınır.

### [C] `config/logic/` — 456 satır
`AdvancedLogicConstants` (korundu), `BasicLogicConstants`, `MidLogicConstants` (silindi).
Üç ayrı mantık seviyesi için ayrı sabit dosyaları.

---

## [C] Robot Settings OpMode — 1.658 satır

| Dosya | Satır |
|---|---|
| `settings/RobotSettingsManagerOld.java` | 836 |
| `settings/RobotSettingsManager.java` | 457 |
| `settings/menu/MenuDefinitions.java` | 129 |
| `settings/menu/MenuCategory.java` | 59 |
| `settings/menu/NumberMenuItem.java` | 54 |
| `settings/menu/EnumMenuItem.java` | 48 |
| `settings/menu/BooleanMenuItem.java` | 46 |
| `settings/menu/MenuItem.java` | 29 |

```java
@TeleOp(name = "Robot Settings", group = "A Robot Settings Manager")
public class RobotSettingsManager extends LinearOpMode {
    enum MenuState { MAIN_MENU, AUTO_SETTINGS, HARDWARE_SETTINGS,
                     BASIC_LOGIC_SETTINGS, MID_LOGIC_SETTINGS, ADVANCED_LOGIC_SETTINGS }
    enum PopupType { NONE, SAVE_CONFIRM, RESET_CONFIRM }
}
```
Kontroller: dpad ↑↓ gez, ←→ değer değiştir, A alt menü/toggle, B geri,
START kaydet, BACK varsayılana dön.

Veri-güdümlü menü mimarisi (`MenuItem` soyut tipi + Boolean/Number/Enum türevleri +
`MenuDefinitions` tanım tablosu) doğru tasarlanmış — 365 satırlık `menu/` paketi
yeniden kullanılabilir.

> ⚠️ **Tuna: "robot main ayarlarını yarışmada kullanmıyorduk."**
> Ayrıca [C] — test kanıtı yok. `RobotSettingsManagerOld` (836) ile birlikte
> 1.293 satırlık iki nesil.
>
> **Bu sezon: Panels + FTC Dashboard ikisi de kurulu, `@Config` anotasyonu bu işi
> yapıyor.** Yazma. Gerçekten gamepad'den ayarlanması gereken 2-3 şey için
> `TelemetryManager` (`07-teleop-controllers.md`) tarzı mini menü yeter.

---

## [C] `iface/` — 669 satır ⭐ *simülasyon köprüsü*

| Dosya | Satır | Ne |
|---|---|---|
| `RobotIntent.java` | 282 | Robotun "ne yapmak istediği" |
| `RobotState.java` | 241 | Robotun tam durumu (sensörler, pozlar) |
| `TaskStatus.java` | 84 | Görev durumu |
| `RobotAction.java` | 62 | Çıkış eylemi (motor komutları) |

**Bu paket `re-cock-nize` simülasyonunun bağlandığı yer.** Java robot mantığı
`RobotState` alıp `RobotAction` üretiyor; simülasyon bunu JPype ile çağırıyor.
Detay → `10-simulation-replay.md`.

**Donanımdan arındırılmış mantık katmanı** — simülasyon istiyorsak bu ayrım şart.

---

## Bu sezon için öneri

### Interface'leri koru — ama kuyruğu kes

**Tut:**
```
subsystems/shooter/ShooterInterface.java
subsystems/shooter/ShooterPidfSubsystem.java
subsystems/shooter/ShooterFactory.java      ← 25 satır, sorun değil
```

**Tutma:** `Settings` + `Storage` + `Constants` üçlüsünü tek dosyaya indir.
Kalibre sayılar için `@Config` anotasyonlu tek `Tuning.java` yeter — Dashboard'dan
canlı değişir, kalıcı kayıt gerekirse o zaman düşünürüz.

Hedef: **bir subsystem eklemek = 2 dosya** (interface + impl), varyant gelince 3. dosya (factory).
Geçen sezon 5 dosyaydı.

### Seçimi tek kaynaktan yap
Geçen sezonun hatası: `ShooterFactory` → `HardwareSettings`, `TurretFactory` →
`HardwareConstants`. **Tek yer olsun.** Basit hâli:
```java
public final class Variants {
    public static ShooterVariant shooter = ShooterVariant.PIDF;
    public static TurretVariant  turret  = TurretVariant.PID;
}
```
Kodda düzenle, derle, yükle. Dashboard'dan canlı değiştirme ihtiyacı gerçekten
doğarsa `@Config` ekle — ama o zaman **tüm** factory'ler oradan okusun.

### Diğer
- `Alliance` enum'unu taşı (40 satır).
- `Chassis` boyutları + birim dönüşüm yardımcılarını taşı; motor↔tekerlek RPM ayrımını koru.
- `iface/` ayrımını (State/Action) simülasyon istiyorsan baştan kur — sonradan
  eklemek çok daha pahalı.
- Robot Settings OpMode'unu yazma, Dashboard kullan.

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
sed -n '797,827p' $P/config/HardwareConstants.java     # SubsystemSelection
cat $P/config/Alliance.java
git show d7711d0^:$P/hardware/subsystems/turret/TurretFactory.java
git show d7711d0^:$P/settings/storage/settings/HardwareSettings.java
git show d7711d0^:$P/settings/RobotSettingsManager.java
git checkout d7711d0^ -- $P/iface/                      # simülasyon arayüzleri
```
