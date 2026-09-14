# Geçen Sezon Mirası — DECODE 2025-26 → BIOBUZZ 2026-27

Bu klasör, `de-cock/robot-code` deposunda geçen sezon yazılan **~70.800 satır** kodun
konu konu envanteridir. Amaç: bu sezon neyi devralacağımıza **kanıta bakarak** karar vermek.

## Neden bu dosyalar var

Geçen sezonun son commit'i (`d7711d0`, 16 Nis 2026) şunu yaptı:
`remove unused code outside LC5 dependency closure` → **289 Java dosyası, 60.060 satır silindi.**
Geriye yarışmada koşan 49 dosya / 10.751 satır kaldı.

Silinen kod git geçmişinde duruyor ve içinde bu sezon işimize yarayacak şeyler var.
Ama hepsi eşit değerde değil.

## Kanıt katmanları

Her dosyada bulgular bu dört etiketten biriyle işaretlidir:

| Etiket | Anlamı | Güven |
|---|---|---|
| **[A] MAÇTA KOŞTU** | `contingency/lvbelc5` bağımlılık kapanışı. Gerçek maçlarda çalıştı. | Yüksek — olduğu gibi taşınabilir |
| **[B] ROBOTTA KOŞTU** | Yarışmada değil ama gerçek robotta çalıştı; çıktı verisi diskte duruyor. | Yüksek — taşınabilir |
| **[C] TEST EDİLMEDİ** | Yazıldı, robotta çalıştığına dair hiçbir kanıt yok. | **Düşük — kod olarak değil, tasarım notu olarak kullan** |
| **[D] SİMÜLASYONDA KOŞTU** | `re-cock-nize` sim ortamında çalıştı, gerçek donanımda değil. | Orta — mantık doğru, donanım davranışı bilinmiyor |

> **Tuna'nın uyarısı (14 Eyl 2026):** "contingency dışındaki kodlar test edilmedi, o yüzden
> o kodlardaki mimariler temiz olsa bile take it with a grain of salt."
>
> Bu uyarı ciddiye alınmalı. [C] etiketli kodun bir kısmı mimari olarak *çok* zarif görünüyor
> (çoklu hipotez takibi, EKF, A*+DWA). Zarif görünmesi çalıştığı anlamına gelmiyor.
> Geçen sezonun hikâyesi tam olarak budur: gelişmiş katman (`logic/advanced`) hazır olmadı,
> yerine acil durum katmanları (`contingency/level0..5`) yazıldı, maçta **level 5** koştu.

## Gall's Law notu

> "Çalışan karmaşık sistem, daima çalışan basit bir sistemden evrilmiş olarak bulunur.
> Sıfırdan tasarlanan karmaşık sistem asla çalışmaz."

Geçen sezonun `contingency/lvbelc5` adının kendisi bu yasanın tutanağıdır: "contingency"
= acil durum, "level 5" = beşinci yeniden yazım. Bu sezonun birincil hedefi bunu tekrarlamamak.

**Kural:** Kapatılan özellik silinir. `if (false && ...)` yerine `git revert`.
Geçen sezonun en büyük teknik borcu buydu (bkz. `07-teleop-controllers.md`).

## Konu dosyaları

| Dosya | Konu | Silinen satır |
|---|---|---|
| `01-shooter.md` | Shooter subsystem, PIDF, atış motorları, balistik | ~1.500 + harici repolar |
| `02-turret.md` | 5 taret varyantı, hall kalibrasyon, tarama modu | ~6.900 |
| `03-tuning-framework.md` | `tunaing/` — otomatik PID/FF tuning çatısı | ~12.700 |
| `04-localization.md` | Lokalizasyon, Limelight, pose yönetimi | ~3.100 |
| `05-vision-ball-tracking.md` | Cartographer, top takibi, HuskyLens, sensör ızgarası | ~6.200 |
| `06-drivetrain-motion-auto.md` | Pedro, AutoBuilder, Ashtar hareket planlama | ~4.900 |
| `07-teleop-controllers.md` | Controller katmanı, zone, recovery, telemetri | ~2.000 |
| `08-architecture-settings.md` | Interface/factory/storage, settings menüsü, contingency | ~3.900 |
| `09-intake-feeder.md` | Intake ve feeder varyantları, top sayma | ~2.100 |
| `10-simulation-replay.md` | `re-cock-nize` simülasyon, parrot kayıt/oynatma | ~1.800 + harici repo |
| `11-strategy-logic.md` | Basic/Mid/Advanced mantık katmanları, strateji motoru | ~3.300 |

## Silinen dosyaya nasıl bakılır

```bash
cd /home/shared/projects/archive/ftc/de-cock/robot-code
P=TeamCode/src/main/java/org/firstinspires/ftc/teamcode

# Tek dosya oku
git show d7711d0^:$P/tunaing/ff/FFRunner.java

# Bir klasörü tümden çıkar
git checkout d7711d0^ -- $P/tunaing/     # çalışma ağacına alır (dikkat: commit etme)

# Silinen dosyaların tam listesi
git show --diff-filter=D --name-only d7711d0 | grep '\.java$'
```

`d7711d0^` = temizlik commit'inden bir öncesi, yani her şeyin hâlâ durduğu son hal.

## İlgili harici repolar

| Repo | İçerik | Durum |
|---|---|---|
| `de-cock/ball-auto-istic` | Atış fiziği, PyBullet sim, RK4, iki balistik motor (messi/ronaldinho) | 34 commit, Java kodu üretiyor |
| `de-cock/re-cock-nize` | JPype ile gerçek robot kodunu koşturan sim + RL ortamı | 50 commit, 56 sim log |
| `de-cock/shooter_auto_runs` | Gerçek robot tuning koşularının çıktısı (28 Kas 2025) | **[B] kanıtın kaynağı** |
| `de-cock/shooter-heatmap` | Saha RPM ısı haritası, 9.3 MB CSV | Çıktı PNG'leri mevcut |
