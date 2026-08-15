import open3d as o3d

print("1. Harita yükleniyor...")
# Dosya adını senin haritana göre "map.pcd" olarak ayarladım!
dosya_adi = "map.pcd" 
pcd = o3d.io.read_point_cloud(dosya_adi)

if not pcd.has_points():
    print(f"HATA: '{dosya_adi}' bulunamadı veya içi boş! Dosyanın bu klasörde olduğundan emin ol.")
    exit()

print(f"-> Orijinal haritadaki nokta sayısı: {len(pcd.points)}")

print("\n2. RANSAC algoritması ile zemin aranıyor...")
plane_model, inliers = pcd.segment_plane(distance_threshold=0.2,
                                         ransac_n=3,
                                         num_iterations=1000)

if len(inliers) == 0:
    print("HATA: Bu haritada bir zemin (düzlem) bulunamadı!")
    exit()

print(f"-> Zemin tespit edildi! Silinen zemin noktası sayısı: {len(inliers)}")

print("\n3. Zemin haritadan temizleniyor...")
engeller_bulutu = pcd.select_by_index(inliers, invert=True)

print(f"-> Temizlenmiş haritadaki (sadece engeller) nokta sayısı: {len(engeller_bulutu.points)}")

print("\n4. Yeni harita kaydediliyor...")
yeni_dosya = "temiz_harita.pcd"
o3d.io.write_point_cloud(yeni_dosya, engeller_bulutu)

print(f"-> Başarılı! Tertemiz harita '{yeni_dosya}' adıyla klasöre kaydedildi.")
