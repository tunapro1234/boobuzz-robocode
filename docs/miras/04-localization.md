# 04 — Lokalizasyon: Odometri, Limelight, Kalman Füzyonu, Pose Yönetimi

## Özet

**Bu sezonun en çok değişen konusu.** Geçen sezon odometri + AprilTag füzyonunu elle
yazmak için ~3.100 satır harcandı (EKF, Kalman füzyon, koordinat dönüşümleri) ve maça
girenin bile `USE_KALMAN = false` ile **kapatıldığı** görülüyor.

**Pedro Pathing 3.0 bunu kütüphane içinde çözüyor.** Kendi katmanını yazmadan önce
`com.pedropathing.localization.FusionLocalizer`'ı dene.

---

## ⚡ ÖNCE BUNU OKU: Pedro 3.0 ne getiriyor

```java
package com.pedropathing.localization;

public interface Localizer {
    void setPose(Pose);      default void setX/setY/setHeading(double);
    default Pose pose();     default Twist twist();     default Velocity velocity();
    MotionState state();     void update();             void reset();
    default Map<String,Object> debug();
}

public class FusionLocalizer implements Localizer {
    FusionLocalizer(Localizer base, Pose q, Pose r, Pose p0, int bufferSize);
    void addMeasurement(Pose visionPose, long timestamp);
    void addMeasurement(Pose visionPose, long timestamp, Pose noise);
    static Pose interpolateTransform(Pose, Pose, double);   // gecikme telafisi
}
```

Yani: **taban lokalizatör (odometri) + zaman damgalı vision ölçümü + gürültü matrisleri
+ gecikme interpolasyonu** — geçen sezon `KalmanFusionLocalizer` (349) ve `EkfLocalizer`
(545) ile yazılmaya çalışılan şeyin tamamı.

Hazır taban lokalizatörler (`com.pedropathing.revhub.localizers`):
`PinpointLocalizer`, `OTOSLocalizer`, `OctoQuadLocalizer`,
`ThreeWheelLocalizer`, `ThreeWheelIMULocalizer`, `TwoWheelLocalizer`
+ config sınıfları ve `CustomIMU` / `RevHubIMU` / `Encoder`.

Ayrıca **alliance mirror kütüphanede**:
```java
PoseFactory.degrees().mirrorX(144).of(x, y, heading)
```
Geçen sezon 14 auto dosyasının yarısını üreten Blue/Red kopyalama derdi çözülüyor.

---

## [A] MAÇTA KOŞTU

### `hardware/subsystems/vision/limelight/Limelight.java` — 172 satır
**Korundu, hâlâ yerinde.** Limelight 3A sarmalayıcısı.

Tuttuğu veriler:
```java
xInch, yInch, yawDeg      // MegaTag2 robot pozu
tagCount                   // kaç AprilTag görülüyor
latencyMs                  // ölçüm gecikmesi ← füzyon için kritik
valid                      // poz geçerli mi
camX, camY, camZ           // kamera-uzay konumu
tx, ty                     // hedefin görüntüdeki açısı
```

Önemli metot: `updateRobotOrientation(double ftcHeadingDeg)` — MegaTag2, robotun
heading'ini bilmek zorunda; IMU/odometri heading'i Limelight'a geri besleniyor.
**Bu adım unutulursa MT2 poz çöp çıkar.**

`getLatencyMs()` var — Pedro 3'ün `addMeasurement(pose, timestamp)` API'sine doğrudan
besleneceği için değerli.

### `contingency/lvbelc5/controllers/LocalizerController.java` — 380 satır
Odometri + Limelight füzyonu. **Ama:**

```java
// Robot.java, satır ~48
private static final boolean USE_KALMAN = false;  // true=Kalman, false=Odom
```

**Maça giren hâlinde Kalman kapalıydı.** Yani 380 satırlık füzyon kodu yazıldı, denendi,
güvenilmedi ve saf odometriye dönüldü. Bu, geçen sezonun en net "karmaşıklık borcu" örneği.

Yine de içindeki fikirler kayda değer:

- **`CalibrationModel` enum**: MT2 pozunu düzeltmek için PIXEL / başka modeller.
  Limelight'ın ham MT2 çıktısı sistematik hata veriyordu, kalibrasyon tablosuyla
  düzeltiliyordu.
- **Spike reddi**: `getSpikeCount()` — ani sıçrayan vision ölçümleri sayılıp atılıyor.
- **`skipNextDelta`**: poz manuel sıfırlandığında bir sonraki odometri delta'sı atlanıyor.
  Reset anındaki sahte sıçramayı engelliyor. **Küçük ama kritik detay.**
- **Koordinat dönüşümü**: `ftcToPedro()` / `pedroHeadingToFtc()` — FTC saha koordinatı
  ile Pedro koordinatı arasında çeviri. Bu sezon Pedro 3'te koordinat sistemi değişti,
  yeniden yapılacak.
- **BACK tuşuyla reset**: `handleBackButtonReset()` — sürücü sahada poz sıfırlayabiliyor.
- **`hardReset(Pose)`**: hem Kalman hem odometri birlikte sıfırlanıyor.

### `settings/storage/PoseStorage.java`
Auto → Teleop poz aktarımı. Statik alanlar, OpMode'lar arası yaşıyor:
```java
storedPose, storedTurretAngleDeg, storedStartLocation,
storedRedAlliance, explicitlySet, initialized
```
`isExplicitlySet()` ayrımı önemli: poz gerçekten set edildi mi, yoksa varsayılan mı —
`Robot.java` buna bakıp auto'dan gelen pozu mu yoksa varsayılan başlangıcı mı kullanacağına
karar veriyor.

**Taret açısını da taşıması** iyi düşünülmüş: auto biterken taret nerede kaldıysa
teleop oradan devam ediyor.

**Bu sezon: doğrudan taşınır.** 

### `settings/storage/vision/LimelightCalibrationStorage.java` — 12 sütunlu veri şeması
```
IDX_ODOM_X/Y/HEADING       (0,1,2)   odometri referansı
IDX_MT2_X/Y/HEADING        (3,4,5)   Limelight MegaTag2 çıktısı
IDX_CAM_X/Y/Z              (6,7,8)   kamera-uzay
IDX_TX, IDX_TY             (9,10)    görüntü açıları
IDX_TAG_AREA               (11)      tag alanı (mesafe vekili)
mt2OffsetX, mt2OffsetY               sistematik düzeltme
```
**Bu şema değerli.** Vision kalibrasyon verisini toplamanın doğru formatı: her örnekte
hem odometri "gerçeği" hem vision ölçümü hem ham görüntü metrikleri. Mesafeye/açıya
bağlı hata modeli buradan çıkarılıyor.

### `PoseResetBlue.java` / `PoseResetRed.java` (LC5 teleop)
Sahada robotu bilinen bir noktaya koyup pozu sıfırlayan küçük OpMode'lar.
Basit ve gerekli. **Bu sezon da lazım** — ama tek OpMode + alliance parametresi olarak.

### Teleop içi hard reset (`BlueTeleop.handleHardReset`)
START + Y **2 saniye basılı** → poz duvara göre sıfırlanır:
```java
new Pose(144 - halfWidth, 0 + halfLength, Math.toRadians(90))
```
Chassis boyutlarından yararlanıp robotu köşeye dayayarak sıfırlama. Yarışmada
lokalizasyon kaçtığında kurtarıcı. **Fikri taşı.**

---

## [C] TEST EDİLMEDİ — `logic/advanced/engines/localizers/` 1.251 satır

| Dosya | Satır | Ne |
|---|---|---|
| `EkfLocalizer.java` | 545 | Genişletilmiş Kalman filtresi, köşegen kovaryans (x, y, heading) |
| `KalmanFusionLocalizer.java` | 349 | Basit Kalman füzyonu |
| `LocalizerInterface.java` | 90 | Arayüz + `CoordinateSystem` enum |
| `FusionLocalizerFactory.java` | 86 | Selector |
| `PedroPassthroughLocalizer.java` | 84 | Sadece Pedro odometrisi (fallback) |
| `FusionLocalizer.java` | 60 | Sarmalayıcı |
| `LocalizerFactory.java` | 37 | Selector |

`EkfLocalizer` yorumu: *"fuses high-rate odometry with intermittent absolute
measurements (e.g. AprilTags)"* — doğru problem tanımı. Kovaryans köşegen tutulmuş
(basitleştirme), süreç/ölçüm gürültüsü `AdvancedLogicConstants.Localization`'dan
geliyor, `CorrectionStrategy` enum'u ile farklı düzeltme yaklaşımları var.

**Mimari temiz ama çalıştığı kanıtlanmadı** — ve zaten `USE_KALMAN=false` ile
LC5'te de kapatıldı. **Pedro 3'ün `FusionLocalizer`'ı bunun yerine geçiyor.**

## [C] Vision kalibrasyon OpMode'ları — `tunaing/opmodes/vision/` 1.885 satır

| Dosya | Satır | Ne |
|---|---|---|
| `LimelightCalibrator.java` | 517 | Odometri-gerçeği ile MT2'yi eşleyip hata modeli çıkarma |
| `LimelightCalibratedTest.java` | 417 | Kalibrasyonlu poz testi |
| `LimelightKalmanTest.java` | 413 | Kalman füzyon testi |
| `LimelightMT1MT2Test.java` | 311 | MegaTag1 vs MegaTag2 karşılaştırması |
| `LimelightAprilTagTest.java` | 227 | Ham AprilTag testi |

Çıktı verisi diskte bulunamadı → [C].

Ama **`LimelightCalibrator`'ın yaptığı iş bu sezon da gerekli**: robotu bilinen
noktalara koy, MT2 ne diyor kaydet, hatayı mesafe/açıya göre modelle.
12 sütunlu şema (yukarıda) bunun veri formatı.

---

## Bu sezon için öneri

1. **Önce Pedro 3 `FusionLocalizer`'ı dene.** Kendi EKF'ini yazma. Arayüz zaten
   `Localizer`, kendi implementasyonunu takmak istersen kapı açık.

2. **`Limelight.java`'yı taşı**, `updateRobotOrientation()` çağrısını unutma,
   `getLatencyMs()`'i `addMeasurement(pose, timestamp)`'a bağla.

3. **`PoseStorage`'ı taşı** — 100 satırın altında, her sezon lazım, `explicitlySet`
   ayrımı ve taret açısı taşıma detaylarıyla birlikte.

4. **`skipNextDelta` fikrini taşı** — poz reset sonrası sahte odometri sıçramasını
   engelleyen o küçük bayrak.

5. **Kalibrasyon şemasını (12 sütun) taşı**, kalibratör OpMode'unu yeniden yaz.

6. **Hard reset + saha reset OpMode'unu ilk günden koy.** Tek OpMode, alliance parametreli.

7. **Spike reddini erteleme.** Vision ölçümü sıçradığında robotun sahada zıplaması
   maç kaybettirir; basit bir "son pozdan X inçten fazla uzaksa reddet" kontrolü ucuz.

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
# Korunan dosyalar (doğrudan oku):
cat $P/hardware/subsystems/vision/limelight/Limelight.java
cat $P/settings/storage/PoseStorage.java
cat $P/contingency/lvbelc5/controllers/LocalizerController.java
# Silinmiş olanlar:
git show d7711d0^:$P/logic/advanced/engines/localizers/EkfLocalizer.java
git show d7711d0^:$P/tunaing/opmodes/vision/LimelightCalibrator.java
```
