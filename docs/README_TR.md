# Ayrık zamanlı araba-sarkaç çalışması

Bu depo, **3 Ekim 2026** tarihli çalışmanın MATLAB/Simulink modellerini, kaynak kodlarını, tam çözünürlüklü sonuçlarını ve animasyonunu içerir. Ana modelde çözücü adımı ve denetleyici örnekleme süresi **0,0001 saniyedir**.

Önce [Türkçe ders notunu](../output/pdf/Ayrik_Araba_Sarkac_Ders_Notu_TR.pdf) okuyun. Düzenlenebilir sürüm [TUTORIAL_TR.md](TUTORIAL_TR.md) dosyasındadır. Sonra [sonuç tablosunu](../study/discrete_results/summary.csv) ve [animasyonu](../study/discrete_results/controllers_2d.mp4) inceleyin.

## Başlatma

Git LFS kurulu olmalıdır. Depoyu klonladıktan sonra `git lfs pull` çalıştırın; aksi halde büyük veri dosyaları yerine küçük işaretçi dosyaları gelebilir.

MATLAB R2018b ve Simulink ile depo kökünden:

```matlab
cd study
addpath(pwd,fullfile(pwd,'KontrolAI'))
results = run_discrete_comparison;
```

Bu komut modelleri yeniden kurar, altı senaryo simülasyonunu çalıştırır, 36 denetleyici yörüngesini kaydeder ve grafiklerle videoyu üretir. Kaydedilmiş sonuçların üzerine yazar; yeniden üretim için ayrı bir klon kullanın.

MATLAB çalıştırmadan arşivi doğrulamak için:

```sh
python tools/verify_archive.py
```

## Dosyalar

- `study/`: çalıştırılabilir kaynaklar ve hazır modeller.
- `study/discrete_results/`: bugünkü tam sonuçlar, MATLAB `.fig` dosyaları, PNG grafikler ve MP4.
- `docs/`: ders notu, yöntem ve yeniden üretim açıklamaları, veri kataloğu.
- `tools/`: SHA-256 doğrulaması ve CSV üzerinden bağımsız metrik hesabı.
- `output/pdf/`: resimli Türkçe ders notu.

Bu deneyde LQR hem sarkaç açısını hem araba konumunu düzenler. Diğer yöntemlerde açının küçük olması, arabanın başlangıç konumuna dönmesi anlamına gelmez. Donanım deneyi, sensör gürültüsü ve fiziksel ray sınırı bu çalışmanın kapsamında değildir.
