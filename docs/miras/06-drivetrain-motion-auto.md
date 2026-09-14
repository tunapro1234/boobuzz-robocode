# 06 — Drivetrain, Hareket Planlama, Otonom

## Özet

Drivetrain tarafı **Pedro Pathing** üzerine kuruluydu ve maçta çalıştı. Otonom için
akıcı bir `AutoBuilder` DSL'i yazılmıştı — **fikir doğru, boyut şişkin** (811 satır).

Ayrıca hiç kullanılmayan bir hareket planlama motoru var: **"Ashtar"** (A* + Dynamic
Window Approach, 3.319 satır) — tamamen test edilmemiş.

> ⚠️ **Pedro 3.0 API'si tamamen değişti.** Aşağıdaki `Constants.createFollower()`
> yaklaşımı artık geçersiz. Karşılıklar bu dosyanın sonunda.

---

## [A] MAÇTA KOŞTU

### `hardware/subsystems/drivetrain/pedroPathing/Constants.java` — 58 satır
Pedro 2.x konfigürasyonu. **Ölçülmüş gerçek robot değerleri** içeriyor:

```java
FollowerConstants:
    mass(13.8)                              // kg
    forwardZeroPowerAcceleration(-36.17)    // güç kesince ileri yavaşlama
    lateralZeroPowerAcceleration(-85.98)    // yanal yavaşlama
    useSecondary{Translational,Heading,Drive}PIDF(false)

MecanumConstants:
    maxPower(1)
    xVelocity(73.63)    // inç/s ileri maksimum
    yVelocity(54.09)    // inç/s yanal maksimum
    motorlar: leftFront, leftBack, rightFront, rightBack
    sol taraf REVERSE, sağ taraf FORWARD

PinpointConstants:
    forwardPodY(161)  strafePodX(0)  birim MM
    goBILDA_4_BAR_POD
    forward FORWARD, strafe REVERSED

PathConstraints(0.99, 100, 1, 1)
```

**Bu sayılar yeni robotta geçersiz** ama **hangi sayıların ölçülmesi gerektiğini**
gösteriyor: kütle, sıfır-güç ivmelenmeleri, eksen maksimum hızları. Pedro'nun tuning
OpMode'larıyla ölçülüyor.

`useSecondary*PIDF(false)` → ikincil (ince ayar) PIDF katmanları kapatılmış.
Basit tutulmuş, çalışmış.

### `contingency/lvbelc5/auto/AutoBuilder.java` — 811 satır ⭐ *fikir taşınmalı*

Akıcı (fluent) otonom DSL'i. Kullanımı:
```java
sequence = AutoBuilder.start(robot, aimingController, Blue.START_MISSIONARY)
    .shoot(3)
    .goToPose(Blue.SCORE_BIG_HIGH)
    .withIntake()
    .goToPose(Blue.PICKUP_HIGH_START)
    .intake(2.0)
    .shoot(3)
    .build();
```

Tam API:
```
Hareket:  goToPose(Pose) · goTo(x,y,headingDeg) · lineTo(x,y) · lineTo(Pose)
          curveTo(Pose end, Pose... ctrl) · curveToPose(...) · turnTo(headingDeg)
Heading:  withTangentialHeading() · withTangentialHeadingReverse()
          withConstantHeading(deg) · withLinearHeading(start,end) · withLinearHeadingTo(end)
Eylem:    shoot(ballCount) · intake(seconds) · waitSeconds(double)
Paralel:  withIntake() · withShooterWarmup()     ← hareketle eşzamanlı
Ayar:     withHoldEnd(boolean) · withPathConstraints(PathConstraints)
```

**Neden değerli:** Otonom yazmayı gerçekten kolaylaştırıyor. `withIntake()` /
`withShooterWarmup()` gibi "hareket ederken şunu da yap" modifierları FTC otonomunda
zaman kazandıran asıl şey.

**Neden 811 satır:** Her hareket metodunun 3-4 aşırı yüklemesi var, heading
interpolasyon varyantları ayrı ayrı yazılmış. **~250 satıra inebilir.**

### Otonom rutinleri — `contingency/lvbelc5/auto/` 2.585 satır
| Dosya | Satır |
|---|---|
| `AutoBuilder.java` | 811 |
| `AutoLocations.java` | ~400 |
| `BlueMissionary9PieceLever.java` / `RedMissionary9PieceLever.java` | 131 ×2 |
| `BlueMissionary9Piece.java` / `RedMissionary9Piece.java` | ~120 ×2 |
| `BlueDoggy6Piece.java` / `RedDoggy6Piece.java` | ~110 ×2 |

**"Missionary" ve "Doggy"** = iki farklı başlangıç pozisyonu/stratejisi.
9 parça ve 6 parça varyantları, "Lever" = ek mekanizma kullanan sürüm.

### `AutoLocations.java` ⭐ *desen taşınmalı*
Saha noktalarının merkezi tanımı. **Mavi tanımlanıyor, kırmızı otomatik aynalanıyor:**
```java
public static final Pose BLUE_HIGH_OUTER = pose(BLUE_OUTER_X, HIGH_Y, 0);
public static final Pose RED_HIGH_OUTER  = mirror(BLUE_HIGH_OUTER);
```
Yardımcılar: `pose(x,y,deg)`, `mirror(Pose)`, `poseFromFrontLeft(...)`,
`poseFromBackLeft(...)` — robotun köşesinden referanslama (duvara dayama pozisyonları için).

İç sınıflar: `Balls` (top konumları), `Blue`, `Red`.

> **Pedro 3 karşılığı:** `PoseFactory.degrees().mirrorX(144)` — elle `mirror()`
> yazmaya gerek yok.

### Blue/Red kopyalama problemi
`BlueMissionary9Piece` ile `RedMissionary9Piece` arasındaki fark **sadece**
`Blue.` → `Red.` ve `Alliance.BLUE` → `Alliance.RED`. 14 otonom dosyasının yarısı böyle.

Aynı durum teleop'ta: `BlueTeleop` vs `RedTeleop` farkı 219 satırda **6 satır**
(alliance sabiti, park hedefi, stick işaretleri).

**Bu sezon: tek OpMode + alliance parametresi.** Pedro 3'ün `PoseFactory.mirrorX()`'i
ile dosya sayısı yarıya iner.

---

## [C] TEST EDİLMEDİ

### "Ashtar" hareket planlama — `logic/advanced/engines/motion/` 3.319 satır

```
MotionController.java (159)              ← arayüz
MotionControllerFactory.java (24)        ← selector
PassthroughMotion.java (450)             ← Pedro'ya doğrudan geçiş (fallback)
ashtar/
  AshtarMotionController.java (540)      ← üst katman
  DynamicWindowApproach.java (431)       ← yerel engelden kaçınma (DWA)
  pathfinding/
    AStarPathfinder.java (435)           ← ızgara üzerinde A*
    GlobalPath.java (228)
    GridNode.java (69)
  planning/
    LocalPlanner.java (290)
    BezierSegment.java (255)
    LocalPlan.java (226)
  execution/
    PathExecutor.java (395)
```

Klasik iki katmanlı mobil robot mimarisi: **global A* ile kaba yol + yerel DWA ile
dinamik engelden kaçınma**, arada Bezier düzleştirme. `OccupancyGrid`'den
(`05-vision-ball-tracking.md`) besleniyor.

**Doğru mimari, ama:** FTC sahasında 2.5 saniyelik otonom hareketler için A*+DWA
ağır kalıyor. Pedro'nun kendi yol takibi zaten var. Hiç sahaya çıkmamış olması
şaşırtıcı değil.

**Bu sezon: taşıma.** Engelden kaçınma gerçekten gerekirse (rakip robot varlığında
otonom navigasyon) o zaman tasarım referansı olarak bak.

### Hareket komutları — `logic/advanced/commands/motion/` 562 satır
| Dosya | Satır |
|---|---|
| `GoToIntakeCommand.java` | 183 |
| `MotionOutput.java` | 148 |
| `GoToShootCommand.java` | 150 |
| `MotionCommand.java` | 46 |
| `MotionCommandFactory.java` | 35 |

### Eski otonom görev sistemi — `auto/` 452 satır [C]
| Dosya | Satır |
|---|---|
| `task/TaskAutoBase.java` | 230 |
| `task/TaskBuilder.java` | 171 |
| `opmode/RedExampleTaskAuto.java` | 51 |

`AutoBuilder`'dan önceki nesil. Terk edilmiş.

### `contingency/level4/auto/AutoBuilder.java` — 359 satır [C]
LC5'teki 811 satırlık sürümün atası. Evrimi görmek için ilginç: 359 → 460 (level5
ilk hâli) → 811 (son hâl). **Her sezon aynı şey oluyor: builder şişiyor.**

### `hardware/subsystems/drivetrain/DriveTrainSubsystem.java` [C]
Pedro öncesi kendi drivetrain sarmalayıcısı.

### `hardware/subsystems/drivetrain/pedroPathing/Tuning.java` [C]
Pedro tuning OpMode'ları.

---

## ⚡ Pedro 2.x → 3.0 geçiş tablosu

| Pedro 2.x (geçen sezon) | Pedro 3.0 (bu sezon) |
|---|---|
| `com.pedropathing.geometry.Pose` | `com.pedropathing.math.Pose` |
| `Constants.createFollower(hwMap)` | `new Follower(localizer, drivetrain, algorithm)` |
| `FollowerBuilder(...).pinpointLocalizer(...).build()` | doğrudan kurucu |
| `follower.followPath(path, true)` | `follower.follow(path)` |
| `follower.getPose()` | `follower.pose()` |
| `follower.setTeleOpDrive(x,y,t,robotCentric)` | `follower.manual(x, y, turn)` |
| `follower.startTeleopDrive()` | `follower.manual(...)` doğrudan |
| `follower.setStartingPose(p)` | `follower.setPose(p)` |
| `follower.isBusy()` | `follower.isBusy()` / `following()` / `idle()` |
| `MecanumConstants` | `MecanumConfig` (`ConfigVar` tabanlı) |
| `PinpointConstants` | `PinpointConfig` |
| `new Path(new BezierLine(a,b))` | `Paths.line(a, b)` / `Paths.curve(...)` / `Paths.through(...)` |
| elle `mirror(Pose)` | `PoseFactory.degrees().mirrorX(144)` |
| — | `FusionLocalizer` (odometri+vision füzyonu hazır) |
| — | `Foresight` algoritması + `ForesightConfig` |

Paket yapısı: `core` (saf matematik, FTC bağımsız) + `revhub` (donanım bağlayıcıları).
`revhub`, `core`'u transitive çekiyor.

Yeni `Follower` API'sinden dikkat çekenler:
```java
follower.hold(Pose)             // noktada tutun
follower.completion()           // yol tamamlanma oranı
follower.remainingDistance()    // kalan mesafe
follower.closestPose()          // yola en yakın nokta
follower.curvature()            // anlık eğrilik
follower.debug()                // FollowerLog
follower.withLogger(Consumer<FollowerLog>)
```

---

## Bu sezon için öneri

1. **Pedro 3 tuning'ini erken yap.** `mass`, `forwardZeroPowerAcceleration`,
   `lateralZeroPowerAcceleration`, `xVelocity`, `yVelocity` — bu 5 sayı ölçülmeden
   yol takibi düzgün çalışmaz.

2. **`AutoLocations` desenini taşı** ama `mirror()`'ı `PoseFactory.mirrorX(144)`'e bırak.

3. **`AutoBuilder`'ı yeniden yaz, ~250 satır.** Metot listesini yukarıdan al, aşırı
   yüklemeleri kırp. `withIntake()` / `withShooterWarmup()` paralel modifier fikrini koru.

4. **Tek OpMode + alliance parametresi.** 14 dosya yerine 4.

5. **Ashtar'ı taşıma.** Pedro'nun kendi takibi yeterli.

6. **`withHoldEnd` ve `PathConstraints` kontrolünü koru** — otonomda nokta nokta
   hassasiyet ayarı lazım oluyor.

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
cat $P/contingency/lvbelc5/auto/AutoBuilder.java       # korundu
cat $P/contingency/lvbelc5/auto/AutoLocations.java     # korundu
cat $P/hardware/subsystems/drivetrain/pedroPathing/Constants.java
git show d7711d0^:$P/logic/advanced/engines/motion/ashtar/AshtarMotionController.java
```
