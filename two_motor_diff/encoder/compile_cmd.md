
## 编译

开启优化：

```
g++ -std=c++17 encoder_dev.cpp -lgpiod -pthread -O2 -o encoder_speed_dev
```

```
g++ encoder_speed_meter.cpp encoder_single.cpp -o encoder_single -lgpiodcxx -lgpiod -pthread -O2
```