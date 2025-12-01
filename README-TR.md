# Mobil VRP Optimizasyon Projesi

Bu proje, Gurobi Optimizer kullanarak Mobil Araç Rotalama Probleminin (Mobil VRP) üç varyasyonunu uygulamaktadır. Problem, bir ana depo, aday mobil depolar (kamyon durakları) ve müşterileri içermektedir. Bir kamyon, malları/personeli aday depolara taşır ve personel bunları müşterilere dağıtır.

## Senaryolar

### Senaryo 1: Minimax Amaçlı Yerleşim-Rotalama Problemi (LRP)

**Dosya:** `project.py`

* **Açıklama**: Bir kamyon Ana Depodan seçilen **tek bir** Aday Depoya gider ve geri döner. Personel seçilen depodan başlar, müşterileri ziyaret eder ve depoya **geri döner** (Kapalı Döngü).
* **Amaç**: Minimize et (Kurulum Maliyeti + Kamyon Gidiş-Dönüş Maliyeti + Maksimum Personel Rota Uzunluğu).
* **Temel Kısıtlar**:
  * Tek depo seçimi.
    *   Kapalı döngü personel rotaları (TSP benzeri).
    *   Personel iş yükü dengelemesi için Minimax amacı.

### Senaryo 2: Açık Yerleşim-Rotalama Problemi (Açık LRP)
**Dosya:** `projectv2.py`
*   **Açıklama**: Senaryo 1'e benzer, ancak personel son müşteriyi ziyaret ettikten sonra depoya **geri dönmez** (Açık Döngü). Bu, personelin başka bir yerden alınabileceği veya vardiyasını son müşteride bitirebileceği senaryoları simüle eder.
*   **Amaç**: Minimize et (Kurulum Maliyeti + Kamyon Gidiş-Dönüş Maliyeti + Maksimum Personel Rota Uzunluğu).
*   **Temel Kısıtlar**:
    *   Tek depo seçimi.
    *   Açık döngü personel rotaları (Yol benzeri).
    *   Minimax amacı.

### Senaryo 3: İki Aşamalı Açık Yerleşim-Rotalama Problemi (2E-Açık LRP)
**Dosya:** `projectv3.py`
*   **Açıklama**: Kamyon, seçilen **bir veya daha fazla** Aday Depoyu ziyaret eden bir **TSP turu** gerçekleştirir. Seçilen her depoda, personel müşterileri ziyaret etmek üzere görevlendirilir ve **geri dönmez** (Açık Döngü).
*   **Amaç**: Minimize et (Kurulum Maliyetleri + Kamyon Rota Mesafesi + Maksimum Personel Rota Uzunluğu).
*   **Temel Kısıtlar**:
    *   Çoklu depo seçimine izin verilir.
    *   Kamyon rotalaması TSP/VRP olarak modellenmiştir.
    *   Açık döngü personel rotaları.
    *   İki aşamalı senkronizasyon.

### Senaryo 4: Heterojen Filo VRP
**Dosya:** `projectv4.py`
*   **Açıklama**: Farklı kapasite ve maliyetlere sahip farklı personel/araç türlerini (örneğin, Kargo Bisikleti ve E-Scooter) tanıtır.
*   **Amaç**: Toplam Maliyeti Minimize Et (Kurulum + Kamyon + Ağırlıklı Mesafe).
*   **Temel Özellikler**:
    *   **Kargo Bisikleti**: Yüksek Kapasite (100), Yüksek Maliyet (1.5/km).
    *   **E-Scooter**: Düşük Kapasite (40), Düşük Maliyet (1.0/km).
    *   Kapasite kısıtları dahildir.

### Senaryo 5: Çok Amaçlı Optimizasyon
**Dosya:** `projectv5.py`
*   **Açıklama**: Ağırlıklı Toplam Yöntemi kullanarak **Operasyonel Maliyet** ile **İş Yükü Dengesi** (Sosyal Adalet) arasında denge kurar.
*   **Amaç**: Minimize et $\alpha \cdot Z_1 + (1 - \alpha) \cdot Z_2$.
*   **Temel Özellikler**:
    *   Pareto Sınırı analizi.
    *   $Z_1$: Toplam Maliyet.
    *   $Z_2$: İş Yükü Dengesizliği ($W_{max} - W_{min}$).

### Senaryo 6: Yeşil VRP (Yüke Bağlı Enerji)
**Dosya:** `projectv6.py`
*   **Açıklama**: Enerji tüketiminin mesafeye ve taşınan yüke bağlı olduğu Elektrikli Araç (EV) fiziğini modeller.
*   **Temel Özellikler**:
    *   Batarya Şarj Durumu (SoC) takibi.
    *   Enerji tüketim formülü: $E = Mesafe \times (Baz + Faktör \times Yük)$.
    *   Araçların negatif olmayan batarya ile dönmesini/bitirmesini sağlar.

### Senaryo 7: VRPTW (Zaman Pencereleri)
**Dosya:** `projectv7.py`
*   **Açıklama**: Müşterilerin belirli zaman pencereleri içinde ziyaret edilmesi gereken zamansal kısıtlamalar ekler.
*   **Temel Özellikler**:
    *   Zaman yayılım mantığı.
    *   Müşterilerde hizmet süreleri.
    *   Kesin zaman penceresi kısıtları.

## Sonuç Özeti

| Senaryo | Toplam Maliyet | Seçilen Depo(lar) | Kamyon Maliyeti | Notlar |
| :--- | :--- | :--- | :--- | :--- |
| **Senaryo 1** | **564.18** | 100 | 300.00 | Personel dönüş yolculukları ve tek depo kısıtlaması nedeniyle yüksek maliyet. |
| **Senaryo 2** | **477.78** | 100 | 300.00 | Dönüş yolculuklarını ortadan kaldırarak Senaryo 1'e göre **~%15 iyileştirme**. |
| **Senaryo 3** | **273.13** | 100, 101 | 176.16 | Senaryo 1'e göre **~%52 iyileştirme**. Kamyon birden fazla depoyu verimli bir şekilde bağlayarak personel için "son mil" mesafesini azaltır. |
| **Senaryo 4** | **820.71** | 100 | 300.00 | Heterojen Filo (Toplam Ağırlıklı Maliyet). Kapasite kısıtları dahil. |
| **Senaryo 5** | *Pareto* | 100 | 300.00 | Maliyet (685-1141) ve Denge (0-342) arasındaki ödünleşim. |
| **Senaryo 6** | **640.93** | 100 | 300.00 | Yeşil VRP. Enerji tüketimi, yüke bağlı olarak rota uzunluğunu sınırlar. |
| **Senaryo 7** | **801.65** | 100 | 300.00 | VRPTW. Katı zaman pencereleri içinde uygun rotalar bulundu. |

## Çıktı Dosyaları

* **Metin Çıktısı**: Ayrıntılı çözücü günlükleri ve rota açıklamaları `output_project*.txt` dosyalarına kaydedilir.
* **Görselleştirmeler**: Rota çizimleri `plot_project*.png` olarak kaydedilir. Son sürümler, daha iyi okunabilirlik için **ızgaralar, yön okları ve net etiketler** içerir.

## Gereksinimler

* Python 3.x
* `gurobipy` (Gurobi Optimizer)
* `matplotlib`
