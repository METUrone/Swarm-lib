# Vaziyet

## Problemler

- PID ile yoldaki her ara node'a gitmeye çalışmak biraz yavaş (özellikler node'lar çok yakın ise)
- Node dağılımına bağlı olarak ajanlar uzun bir süre birbirini takip edecek şekilde gidebiliyor. Bu büyük ölçüde yavaşlamaya sebep oluyor.

## Yapılacaklar

- [ ] Hesaplanan hız vektörlerini ROS topic'lerine publish et.
- [ ] Ajanların çarpışma ihtimaline göre beklet ve devam ettir.
- [ ] Çarpışma durumunda bekleme yerine hız azaltılabilir ?
