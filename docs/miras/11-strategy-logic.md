# 11 — Mantık Katmanları: Basic / Mid / Advanced, Strateji Motoru

## Özet

Geçen sezon **üç paralel robot mantığı** yazıldı (`logic/basic`, `logic/mid`,
`logic/advanced`) — artan yetenek seviyelerinde, aynı işi farklı karmaşıklıkta yapan
üç ayrı sistem. Toplam ~3.300 satır (advanced'ın motor/haritalama kısımları hariç).

**Hiçbiri maça girmedi.** Maça `contingency/lvbelc5` girdi — bu üçünden de bağımsız,
daha doğrudan yazılmış bir dördüncü yol.

> Bu dosya büyük ölçüde **"ne yapılmamalı" dersi**. Ama içinde bu sezon işe yarayacak
> üç fikir var: donanımsız mantık katmanı, global durum konteyneri, ve strateji
> soyutlaması.

---

## [C] Üç seviyeli mantık mimarisi — hiçbiri test edilmedi

### `logic/basic/` — 902 satır
| Dosya | Satır |
|---|---|
| `BasicRobotLogic.java` | 303 |
| `task/DriveTaskExecutor.java` | 172 |
| `task/ShootTaskExecutor.java` | 147 |
| `ShootController.java` | 146 |
| `task/PickupTaskExecutor.java` | 134 |

### `logic/mid/` — 1.100 satır
| Dosya | Satır |
|---|---|
| `MidRobotLogic.java` | 303 |
| `ZoneHelper.java` | 198 |
| `task/DriveTaskExecutor.java` | 172 |
| `ShootController.java` | 146 |
| `task/ShootTaskExecutor.java` | 147 |
| `task/PickupTaskExecutor.java` | 134 |

> 🚩 **`basic` ve `mid` neredeyse birebir aynı.** `DriveTaskExecutor` 172↔172,
> `ShootController` 146↔146, `PickupTaskExecutor` 134↔134, `ShootTaskExecutor` 147↔147,
> `*RobotLogic` 303↔303. Fark sadece `mid`'e eklenen `ZoneHelper` (198).
>
> **1.000 satır kopyala-yapıştır.** Aynı hastalık Blue/Red teleop'ta da var
> (`07-teleop-controllers.md`).

Karşılık gelen sabit dosyaları: `config/logic/BasicLogicConstants.java`,
`MidLogicConstants.java` (456 satır, ikisi de silindi).

### `logic/advanced/` — çekirdek 1.298 satır (+ motorlar ayrı dosyalarda)

```
RobotLogic.java (270)              ← ana koordinatör
RobotGlobalContainer.java (134)    ← paylaşılan durum
controllers/
  DrivebaseController.java (291)
  TurretController.java (96)
  ShooterController.java (95)
  SystemController.java (20)
commands/
  targeting/TargetingCommand.java (237)
  strategy/FullAutoCommand.java (83)
  shooter/{ShooterIdleCommand(48), BinaryShooterIdle(44),
           DistanceBasedShooterIdle(59), ShooterIdleCommandFactory(70)}
  turret/  → 02-turret.md
  motion/  → 06-drivetrain-motion-auto.md
engines/
  strategy/{StrategyEngine(60), FabiusStrategy(55), StrategyEngineFactory(48)}
  shot/       → 01-shooter.md
  localizers/ → 04-localization.md
  cartographer/ → 05-vision-ball-tracking.md
  motion/     → 06-drivetrain-motion-auto.md
```

---

## Taşınmaya değer üç fikir

### 1. `RobotLogic` — donanımsız mantık katmanı ⭐⭐
```java
/**
 * Main robot logic coordinator.
 * Hardware-independent - can be used in simulation.
 *
 * Input:  RobotState  (bize gelen state)
 * Output: RobotAction (bizim gönderdiğimiz action)
 */
```
**Bu, simülasyonun mümkün olmasını sağlayan ayrım** (`10-simulation-replay.md`).
Robot mantığı `HardwareMap` görmüyor; durum alıp eylem döndürüyor. `re-cock-nize`
JPype ile tam buraya bağlanıyor.

Mimari açıklaması dosyanın başında net yazılmış:
- `TargetingCommand` motoru sarmalar, hood ve feeder'ı doğrudan kontrol eder
- `ShooterController` / `TurretController` durum makineleriyle `TargetingCommand`'a
  bağlanır veya bağlanmaz

**Tuna simülasyon istiyor → bu ayrım baştan kurulmalı.** Sonradan eklemek çok pahalı.

### 2. `RobotGlobalContainer` — tek-yazıcı/çok-okuyucu durum ⭐
```java
/**
 * Thread-safe, single-writer-multi-reader pattern.
 * Each section has a designated WRITER (only one component updates it)
 * and multiple READERS.
 * volatile fields ensure visibility; no locks needed.
 */
Pose robotPose = RobotGlobalContainer.getPose();
Pose velocity  = RobotGlobalContainer.getVelocity();
```
**Her alanın tek bir yazarı olması** disiplinli bir kural. Kilit gerektirmeden
thread güvenliği sağlıyor, ve daha önemlisi "bu değeri kim güncelliyor?" sorusunun
tek cevabı oluyor.

> ⚠️ Ama global singleton — dikkatli kullanılmazsa gizli bağımlılık üretir.
> Geçen sezon `CartographerInterface` yorumunda *"Robot pose is accessed via
> RobotGlobalContainer.getPose()"* yazıyor; yani bileşenler pozu parametre olarak
> almak yerine global'den çekiyordu. Bu, test etmeyi zorlaştırır.
>
> **Öneri:** Fikri al (tek yazıcı disiplini), global erişimi alma. Pozu parametre geç.

### 3. Strateji soyutlaması — `StrategyEngine` (60) + `FabiusStrategy` (55)
```java
/**
 * Fabius Strategy - Defensive, conservative autonomous cycling.
 * Named after Quintus Fabius Maximus...
 *
 * Decision logic:
 * - Has space for more balls AND balls exist → Go intake
 * - Full OR no more balls on field → Go shoot
 *
 * TODO: Implement conservative decision logic
 */
```
> 🚩 **`TODO: Implement conservative decision logic`** — strateji motoru **hiç
> implement edilmemiş.** 55 satırın içi boş. Tek strateji implementasyonu bu.

Ama karar mantığının kendisi (yorumda yazılı) **doğru ve basit**:
```
yer var VE saha'da top var  → topla
dolu VEYA saha'da top yok   → at
```
İki satırlık bir durum makinesi. Bu sezon tam otonom döngü istenirse başlangıç noktası.
**60 satırlık `StrategyEngine` arayüzüne gerek yok, iki `if` yeter.**

---

## Ne yanlış gitti — bu sezonun dersi

Geçen sezon **aynı işi yapan dört paralel yol** yazıldı:

| Yol | Satır | Maça girdi mi |
|---|---|---|
| `logic/basic` | 902 | ✗ |
| `logic/mid` | 1.100 | ✗ |
| `logic/advanced` | 12.375 | ✗ |
| `contingency/level0..5` | 14.047 | ✓ (sadece level5) |

**~28.000 satır, %22'si sahaya çıktı.**

Sıralama da öğretici: `logic/*` önce yazıldı (sofistike plan), yetişmeyince
`contingency/*` başladı (acil durum planı), ve maça o girdi. Sonra
`contingency` içinde de 6 seviye yazıldı — yani acil durum planının kendisi de
altı kez yeniden yazıldı.

> **Gall's Law:** *"Sıfırdan tasarlanan karmaşık sistem asla çalışmaz ve
> çalıştırılamaz. Basit bir sistemden başlamak zorundasın."*
>
> `logic/advanced` tam olarak "sıfırdan tasarlanan karmaşık sistem"di:
> strateji motoru + haritalama + EKF + A*/DWA hareket planlama + komut katmanı,
> hepsi birden. Hiçbiri sahaya çıkmadı.
>
> `contingency/level0` ise "basit sistem"di ve gerçekten evrildi — level5'e kadar.
> **Doğru olan bu oldu, ama eski seviyeler silinmediği için depo şişti.**

---

## Bu sezon için öneri

1. **Tek yol olsun.** `basic`/`mid`/`advanced` gibi paralel seviyeler açma.
   Bir sistem yaz, evrilt.

2. **`RobotState` → mantık → `RobotAction` ayrımını baştan kur.** Simülasyon
   istiyorsak şart; istemesek bile test edilebilirlik kazandırıyor.

3. **Strateji motoru yazma.** Otonom karar mantığı gerçekten gerekirse iki `if`
   ile başla (`FabiusStrategy` yorumundaki mantık). Arayüz + factory + engine
   üçlüsüne ancak ikinci strateji gerçekten gerektiğinde çık.

4. **Global durum konteyneri yerine parametre geç.** Tek-yazıcı disiplinini koru
   ama `RobotGlobalContainer.getPose()` yerine `logic.update(state)`.

5. **Eski nesli sil.** Yeni bir yaklaşım kazandığında eskisini `git revert`/`git rm`.
   Geçen sezon 6 contingency seviyesi Nisan'a kadar kodda durdu.

6. **"TODO: Implement" ile commit etme.** `FabiusStrategy` 55 satır iskelet olarak
   4 ay durdu. Ya yap ya silme.

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git show d7711d0^:$P/logic/advanced/RobotLogic.java
git show d7711d0^:$P/logic/advanced/RobotGlobalContainer.java
git show d7711d0^:$P/logic/advanced/engines/strategy/FabiusStrategy.java
# basic ile mid'in aynılığını görmek için:
diff <(git show d7711d0^:$P/logic/basic/task/DriveTaskExecutor.java) \
     <(git show d7711d0^:$P/logic/mid/task/DriveTaskExecutor.java)
```
