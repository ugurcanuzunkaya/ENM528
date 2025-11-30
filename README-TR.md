# Mobil VRP Optimizasyon Projesi

Bu proje, Gurobi Optimizer kullanarak Mobil Araç Rotalama Probleminin (Mobil VRP) üç varyasyonunu uygulamaktadır. Problem, bir ana depo, aday mobil depolar (kamyon durakları) ve müşterileri içermektedir. Bir kamyon, malları/personeli aday depolara taşır ve personel bunları müşterilere dağıtır.

## Senaryolar

### Senaryo 1: Minimax Amaçlı Yerleşim-Rotalama Problemi (LRP)

**Dosya:** `project.py`

* **Açıklama**: Bir kamyon Ana Depodan seçilen **tek bir** Aday Depoya gider ve geri döner. Personel seçilen depodan başlar, müşterileri ziyaret eder ve depoya **geri döner** (Kapalı Döngü).
* **Amaç**: Minimize et (Kurulum Maliyeti + Kamyon Gidiş-Dönüş Maliyeti + Maksimum Personel Rota Uzunluğu).
* **Temel Kısıtlar**:
  * Tek depo seçimi.
  * Kapalı döngü personel rotaları (TSP benzeri).
  * Personel iş yükü dengelemesi için Minimax amacı.

### Senaryo 2: Açık Yerleşim-Rotalama Problemi (Açık LRP)

**Dosya:** `projectv2.py`

* **Açıklama**: Senaryo 1'e benzer, ancak personel son müşteriyi ziyaret ettikten sonra depoya **geri dönmez** (Açık Döngü). Bu, personelin başka bir yerden alınabileceği veya vardiyasını son müşteride bitirebileceği senaryoları simüle eder.
* **Amaç**: Minimize et (Kurulum Maliyeti + Kamyon Gidiş-Dönüş Maliyeti + Maksimum Personel Rota Uzunluğu).
* **Temel Kısıtlar**:
  * Tek depo seçimi.
  * Açık döngü personel rotaları (Yol benzeri).
  * Minimax amacı.

### Senaryo 3: İki Aşamalı Açık Yerleşim-Rotalama Problemi (2E-Açık LRP)

**Dosya:** `projectv3.py`

* **Açıklama**: Kamyon, seçilen **bir veya daha fazla** Aday Depoyu ziyaret eden bir **TSP turu** gerçekleştirir. Seçilen her depoda, personel müşterileri ziyaret etmek üzere görevlendirilir ve **geri dönmez** (Açık Döngü).
* **Amaç**: Minimize et (Kurulum Maliyetleri + Kamyon Rota Mesafesi + Maksimum Personel Rota Uzunluğu).
* **Temel Kısıtlar**:
  * Çoklu depo seçimine izin verilir.
  * Kamyon rotalaması TSP/VRP olarak modellenmiştir.
  * Açık döngü personel rotaları.
  * İki aşamalı senkronizasyon.

## Sonuç Özeti

| Senaryo | Toplam Maliyet | Seçilen Depo(lar) | Kamyon Maliyeti | Notlar |
| :--- | :--- | :--- | :--- | :--- |
| **Senaryo 1** | **584.18** | 100 | 300.00 | Personel dönüş yolculukları ve tek depo kısıtlaması nedeniyle yüksek maliyet. |
| **Senaryo 2** | **497.78** | 100 | 300.00 | Dönüş yolculuklarını ortadan kaldırarak Senaryo 1'e göre **~%15 iyileştirme**. |
| **Senaryo 3** | **303.13** | 100, 101 | 176.16 | Senaryo 1'e göre **~%48 iyileştirme**. Kamyon birden fazla depoyu verimli bir şekilde bağlayarak personel için "son mil" mesafesini azaltır. |

## Çıktı Dosyaları

* **Metin Çıktısı**: Ayrıntılı çözücü günlükleri ve rota açıklamaları `output_project.txt`, `output_projectv2.txt` ve `output_projectv3.txt` dosyalarına kaydedilir.
* **Görselleştirmeler**: Rota çizimleri `plot_project.png`, `plot_projectv2.png` ve `plot_projectv3.png` olarak kaydedilir.

## Gereksinimler

* Python 3.x
* `gurobipy` (Gurobi Optimizer)
* `matplotlib`
