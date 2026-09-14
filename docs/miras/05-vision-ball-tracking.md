# 05 — Vision & Top Takibi: Cartographer, HuskyLens, Sensör Izgarası

> ⚠️ **Bu sezon için en kritik konu** (Tuna: "top algılama bu sezon çok önemli,
> dağınık olacak çünkü toplar"). Aynı zamanda **en az kanıtlanmış** konu.
> Buradaki 4.399 satırlık `cartographer` mimari olarak etkileyici ama robotta
> çalıştığına dair hiçbir iz yok. Tasarım notu olarak oku, kod olarak kopyalama.

---

## [A] MAÇTA KOŞTU — çok az şey

Maça giren vision: sadece **Limelight AprilTag lokalizasyonu** (`04-localization.md`).
**Top algılama maça hiç girmedi.** LC5'te top tespiti yok; intake'teki mesafe sensörü
ile "topum var mı" seviyesinde kaldı (`09-intake-feeder.md`).

Bu önemli bir gerçek: geçen sezon 4.399 satırlık haritalama sistemi yazıldı,
**hiçbiri sahaya çıkmadı.**

---

## [C] `cartographer/` — 4.399 satır, hepsi test edilmemiş

Proje adı **"Piri Reis"** (haritacı). Mimarisi katmanlı:

```
CartographerInterface (217)          ← arayüz
├── PassthroughCartographer (190)    ← boş implementasyon (fallback)
└── PiriReisCartographer (422)       ← gerçek implementasyon
    ├── BallTracker (619)            ← top tespiti + takip
    ├── RobotTracker (212)           ← rakip robot takibi
    ├── SensorProcessor (202)        ← mesafe sensörü → harita
    ├── OccupancyGrid (404)          ← olasılıksal engel haritası
    └── ScanCoverageMap (383)        ← "nereyi taradık"
tracking/
    ├── MultiHypothesisTracker (560)
    ├── HungarianAlgorithm (443)
    └── KalmanFilter2D (349)
TrackedBall (139) / TrackedRobot (222) / CartographerFactory (37)
```

### `OccupancyGrid.java` — 404 satır ⭐ *fikir değerli*
Saha-merkezli olasılıksal doluluk ızgarası, **log-odds Bayesçi güncelleme** ile:
```java
FIELD_SIZE_INCH = 144.0
L_OCC  =  0.85   // dolu gözlemi
L_FREE = -0.4    // boş gözlemi
L_MIN  = -2.0    // alt sınır
L_MAX  =  3.5    // üst sınır
```
Hücre değeri 0.0 (kesin boş) – 0.5 (bilinmiyor) – 1.0 (kesin dolu).
Sensör okuması olasılığı **değiştirmiyor, güncelliyor** — çoklu gözlem güveni artırıyor,
eski gözlemler zamanla sönümleniyor.

Bu, robotikte standart ve doğru yaklaşım. FTC ölçeğinde 144×144 inç ızgarayı tutmak
hafıza açısından da makul.

> Kodda duran TODO: *"Gamepad rumble — robot yüksek olasılıklı dolu hücreye yaklaşınca
> haptik geri bildirim ver."* Güzel fikir, yapılmamış.

### `ScanCoverageMap.java` — 383 satır ⭐ *bu sezon çok işe yarar*
"Nereleri taradık, nereleri bilmiyoruz?" haritası. Her hücre için:
```java
long[][]   lastScanTimeMs    // son ne zaman bakıldı
double[][] scanConfidence    // tarama güveni 0-1
double     halfLifeMs        // güven yarı ömrü — eski taramalar sönümleniyor
```
İki sensör kaynağını modelliyor:
- **HuskyLens** (taret üstünde) → taret açısına göre FOV konisi
- **SensorGrid** (şasi üstünde) → şasi açısına göre sensör yönleri

**Neden değerli:** Kör sinüs taraması yerine "en uzun süredir bakılmayan yöne dön"
stratejisi kurulabilir. Toplar dağınıksa arama verimliliği doğrudan puana dönüşür.

### `BallTracker.java` — 619 satır
Vision tespiti → kalıcı top takibi. Üç teknik birleştiriyor:
- Kalman filtresi ile pozisyon/hız yumuşatma
- **Macar algoritması** (`HungarianAlgorithm`, 443 satır) ile tespit↔iz optimal eşleme
- **Çoklu hipotez takibi** (`MultiHypothesisTracker`, 560 satır) belirsiz eşlemelerde

### `TrackedBall.java` — 139 satır
```java
enum BallColor { RED, BLUE, NEUTRAL, UNKNOWN }
double x, y;          // saha koordinatı (inç)
double vx, vy;        // hız (inç/s)
BallColor color;
double confidence;    // 0-1
long lastSeenMs;
int trackingId;       // kareler arası kimlik
```
Hız alanları **kesişme (intercept) hesabı** için düşünülmüş — hareketli topu
yakalamak. Bu sezon toplar dağınık ve muhtemelen hareketliyse işe yarar.

### `MultiHypothesisTracker.java` — 560 satır
Dosya başındaki yorum dürüst ve iyi yazılmış: MHT'nin avantajları (tıkanma/occlusion
toleransı, geçici yanlış eşlemeden kurtulma, gecikmeli karar) ve sınırları (üstel
hipotez büyümesi → budama, hafıza → max hipotez sınırı, FTC zaman kısıtı → basitleştirilmiş
skorlama) açıkça yazılmış.

**Dürüst değerlendirme:** Bu, FTC için fazlasıyla gelişmiş. 3-4 top izlemek için
Macar algoritması + MHT gerekmez. Basit en-yakın-komşu eşleme + zaman aşımı yeterli
olurdu. **Gall's Law'un bu sezon en çok ısıracağı yer burası.**

### `RobotTracker.java` (212) / `SensorProcessor.java` (202) / `PiriReisCartographer.java` (422)
Rakip robot takibi, mesafe sensörü verisini haritaya işleme, hepsini birleştiren üst katman.

---

## [C] Vision donanım katmanı — `hardware/subsystems/vision/` 990 satır

| Dosya | Satır | Ne |
|---|---|---|
| `huskylens/HuskyLensSubsystem.java` | 270 | HuskyLens kamera subsystem |
| `huskylens/HuskyLensDevice.java` | 265 | Alçak seviye I2C sürücü |
| `CameraConversion.java` | 231 | **Görüntü koordinatı → saha koordinatı projeksiyonu** |
| `DetectedObject.java` | 126 | Ham tespit |
| `ProjectedObject.java` | 88 | Saha koordinatına yansıtılmış tespit |
| `VisionSource.java` | 10 | Arayüz |

**`CameraConversion.java` (231) en çok işe yarayacak parça.** Kameranın gördüğü
piksel/açıyı, kamera yüksekliği ve eğimini kullanarak zemindeki saha noktasına
çevirme matematiği. Kamera değişse de matematik aynı.

`VisionSource` 10 satırlık arayüz — HuskyLens/Limelight/başkası arasında geçişi
ucuzlatmak için. Doğru soyutlama seviyesi.

### `settings/storage/vision/HuskyLensCalibrationStorage.java` — 82 satır [C]
Renk kalibrasyon verisi.

### `tunaing/opmodes/vision/HuskyLensAutoTune.java` (438) + `HuskyLensColorTest.java` (92) [C]
Renk eşiği otomatik ayarı. Çıktı verisi diskte yok.

---

## [C] Sensör ızgarası — `hardware/subsystems/sensorgrid/` 314 satır

| Dosya | Satır |
|---|---|
| `SensorGridSubsystem.java` | 198 |
| `PointCloud.java` | 116 |

Şasi çevresine dizilmiş mesafe sensörlerinden nokta bulutu üretip engel haritasına
besleme. İlgili sürücüler `test/sensors/` altında (2.681 satır, hepsi [C]):

| Dosya | Satır | Ne |
|---|---|---|
| `VL53L1X_Advanced.java` | ~ | ToF mesafe sensörü, gelişmiş mod |
| `VL53L1XDriver.java` | ~ | ToF sürücü |
| `VL53L0XDriver.java` / `VL53L0XTest.java` | ~ | Eski nesil ToF |
| `TCA9548ADriver.java` / `TCA9548ATest.java` | ~ | **I2C çoklayıcı** — birden çok aynı adresli sensör |
| `DistanceSensorI2CTest.java` / `DistanceSensorREVTest.java` | ~ | REV mesafe sensörü |
| `HuskyLensColorTest.java` | ~ | |
| `LimelightLocalizationTest.java` | ~ | |
| `ThroughboreEncoderTest.java` | ~ | |

**`TCA9548A` çoklayıcı sürücüsü pratik değer taşıyor** — FTC'de aynı I2C adresli
birden fazla ToF sensör kullanmanın standart çözümü. Donanım aynıysa doğrudan taşınır.

### İlgili Python araçları (`de-cock/robot-code/tools/`)
`calibrate_sensor_grid.py`, `calibrate_huskylens.py`

---

## İlgili: `de-cock/ball-auto-istic` top renk kalibrasyonu
```
calibration/tools/calibrate_ball_color.py
calibration/tools/annotate_ball.py
calibration/color_config.json
```
Video/foto üzerinde top işaretleyip renk eşiği çıkarma akışı. **Bu sezon doğrudan
kullanılabilir** — yeni topun rengini kalibre etmek için hazır araç.

---

## Bu sezon için öneri

**Kademeli plan — her adım çalışır hâlde bitmeli:**

1. **Adım 0 — Görüyor mu?**
   Kamera bir top görüyor, ekranda `tx, ty` veriyor. Başka hiçbir şey yok.

2. **Adım 1 — Nerede?**
   `CameraConversion` mantığıyla saha koordinatına yansıt. Tek top, takip yok,
   filtre yok. Telemetride `(x, y)` göster, sahada metreyle doğrula.

3. **Adım 2 — Sürebiliyor mu?**
   En yakın topa git. Pedro `follow(Paths.line(pose(), ballPose))`. Hâlâ takip yok.

4. **Adım 3 — Takip.** Ancak Adım 2 sahada çalışıyorsa:
   en-yakın-komşu eşleme + `lastSeenMs` zaman aşımı. ~80 satır.
   **Macar algoritması ve MHT'ye gitme.**

5. **Adım 4 — Arama.** `ScanCoverageMap` fikri: en uzun süredir bakılmayan bölgeye dön.
   Tarama mantığı `02-turret.md`.

6. **`OccupancyGrid`'i ancak engelden kaçınma gerçekten gerekirse aç.**

**Ne taşınır:**
- `CameraConversion.java` mantığı (231) — matematiği yeniden türet, sayıları kontrol et
- `TrackedBall` veri şeması (139) — sade, doğru alanlar
- `VisionSource` arayüzü (10)
- `TCA9548ADriver` — donanım aynıysa
- `ball-auto-istic/calibration/tools/` renk kalibrasyon akışı

**Ne taşınmaz:**
- `MultiHypothesisTracker` (560), `HungarianAlgorithm` (443) — FTC ölçeği için aşırı
- `PiriReisCartographer` + `OccupancyGrid` — gerçek ihtiyaç doğana kadar

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git show d7711d0^:$P/hardware/subsystems/vision/CameraConversion.java
git show d7711d0^:$P/logic/advanced/engines/cartographer/ScanCoverageMap.java
git show d7711d0^:$P/logic/advanced/engines/cartographer/TrackedBall.java
git checkout d7711d0^ -- $P/logic/advanced/engines/cartographer/   # tümünü incelemek için
```
