# middle_lane_cost_layer

ROS 2 Humble / Nav2 için özel global costmap katmanı.

## Ne yapar?

`config/road_segments_50_points.csv` içindeki 50 noktayı şu şekilde **25 bağımsız orta şerit segmentine** ayırır:

- 1 -> 2
- 3 -> 4
- 5 -> 6
- ...
- 49 -> 50

**2 -> 3, 4 -> 5, 6 -> 7 gibi bağlantılar oluşturulmaz.**

Yalnızca bu segmentlerin `lane_half_width` kadar yakınındaki costmap hücrelerine `lane_cost` uygulanır.
Sağ taraf, sol taraf veya yolun geri kalanı için bias uygulanmaz.
Mevcut obstacle maliyetleri azaltılmaz.

## Kurulum

Paketi workspace'in `src` dizinine çıkarın:

```bash
cd ~/ros2_ws/src
unzip middle_lane_cost_layer.zip

cd ~/ros2_ws
colcon build --symlink-install --packages-select middle_lane_cost_layer
source install/setup.bash
```

Plugin'in görüldüğünü kontrol edin:

```bash
ros2 plugin list | grep MiddleLaneCostLayer
```

Beklenen:

```text
middle_lane_cost_layer::MiddleLaneCostLayer
```

## Nav2 YAML

Global costmap plugin sıralamasına ekleyin:

```yaml
plugins:
  - static_layer
  - obstacle_layer
  - inflation_layer
  - middle_lane_cost_layer
```

Ardından:

```yaml
middle_lane_cost_layer:
  plugin: "middle_lane_cost_layer::MiddleLaneCostLayer"
  enabled: true
  csv_path: "/home/otonom/ros2_ws/install/middle_lane_cost_layer/share/middle_lane_cost_layer/config/road_segments_50_points.csv"
  lane_half_width: 0.35
  lane_cost: 230
```

Eski `right_side_bias_layer` artık kullanılmayacaksa global costmap `plugins` listesinden kaldırın.

## Parametreler

- `enabled`: katmanı açar/kapatır.
- `csv_path`: 50 noktalı CSV dosyasının yolu.
- `lane_half_width`: orta şeridin her iki tarafındaki yüksek-cost kalınlığı. `0.35` => toplam yaklaşık `0.70 m`.
- `lane_cost`: orta şerit hücrelerinin cost değeri. Güvenli aralık `0..252`, varsayılan `230`.

## Log

Başarılı yüklemede yaklaşık şu log görünür:

```text
loaded 25 independent middle-lane segments
pairing mode: (1-2), (3-4), (5-6), ...
```
