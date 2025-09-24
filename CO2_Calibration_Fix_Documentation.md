# SEN66 CO2 Calibration NACK Error Fix Documentation

## ปัญหาที่พบ
- เมื่อทำการ calibrate CO2 ด้วย SEN66 sensor จะได้รับ error "Received NACK on transmit of address"
- ปัญหานี้เกิดจากการรบกวนของสัญญาณ I2C จากจอ OLED display ที่ใช้ I2C bus เดียวกัน

## การแก้ไข
### 1. เพิ่มตัวแปรสำหรับจัดเก็บสถานะจอแสดงผล
```cpp
static int originalDisplayBrightness = -1;
```

### 2. ปรับปรุงฟังก์ชัน `performSen66ForcedCo2Calibration()`

## Flow การทำงานหลังการแก้ไข

### ขั้นตอนที่ 1: เตรียมระบบ
1. **ตรวจสอบสถานะ I2C Exclusive**
   - ถ้า `sen66ExclusiveI2c = true` แสดงว่ามีการใช้งาน I2C อยู่แล้ว → จบการทำงาน

2. **ปิดจอ OLED Display**
   ```cpp
   originalDisplayBrightness = configuration.getDisplayBrightness();
   oledDisplay.setBrightness(0);
   ```
   - เก็บค่า brightness เดิมไว้
   - ปิดจอแสดงผลเพื่อหยุดการรบกวน I2C

3. **รอให้ I2C Bus เสถียร**
   ```cpp
   delay(3000);
   ```
   - รอ 3 วินาที เพื่อให้ I2C bus มีเวลาทำความสะอาด

### ขั้นตอนที่ 2: รีเซ็ต I2C Bus
```cpp
Wire.end();
delay(100);
Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
delay(500);
```
- ปิดและเปิด I2C bus ใหม่เพื่อล้างสถานะที่อาจติดค้าง
- รอ 0.5 วินาที ให้ I2C พร้อมใช้งาน

### ขั้นตอนที่ 3: ล็อค I2C และหยุด Sensor
```cpp
sen66ExclusiveI2c = true;
sen66PollingSuspended = true;
sen66Sample.valid = false;
```
- ล็อค I2C ไม่ให้ process อื่นใช้งาน
- หยุดการอ่านค่าจาก sensor ชั่วคราว

### ขั้นตอนที่ 4: ตรวจสอบสถานะ Sensor
- ถ้า `sen66Ready = false` → คืนค่าจอแสดงผลและจบการทำงาน

### ขั้นตอนที่ 5: หยุดการวัดค่า
```cpp
sen66.stopMeasurement();
```
- หยุดการวัดค่าต่อเนื่องของ SEN66
- ถ้าล้มเหลว → คืนค่าจอแสดงผลและจบการทำงาน

### ขั้นตอนที่ 6: เตรียมทำ Calibration
```cpp
delay(5000);
```
- รอ 5 วินาที เพื่อให้ sensor พร้อมสำหรับ calibration

### ขั้นตอนที่ 7: ทำ CO2 Calibration (พร้อม Retry)
```cpp
for (int attempt = 1; attempt <= 3; attempt++) {
    frcErr = sen66.performForcedCo2Recalibration(400, correction);
    if (frcErr == NO_ERROR) {
        // สำเร็จ → ออกจาก loop
        break;
    } else {
        // ล้มเหลว → รอ 2 วินาที แล้วลองใหม่
        if (attempt < 3) {
            delay(2000);
        }
    }
}
```
- ลองทำ calibration สูงสุด 3 ครั้ง
- ถ้าครั้งแรกล้มเหลว รอ 2 วินาที แล้วลองใหม่
- แสดงสถานะความพยายาม (attempt 1/3, 2/3, 3/3)

### ขั้นตอนที่ 8: รีเซ็ต Device (พร้อม Retry)
```cpp
for (int attempt = 1; attempt <= 3; attempt++) {
    resetErr = sen66.deviceReset();
    if (resetErr == NO_ERROR) {
        // สำเร็จ → ออกจาก loop
        break;
    } else {
        // ล้มเหลว → รอ 1 วินาที แล้วลองใหม่
        if (attempt < 3) {
            delay(1000);
        }
    }
}
```
- ลองรีเซ็ต device สูงสุด 3 ครั้ง
- ถ้าครั้งแรกล้มเหลว รอ 1 วินาที แล้วลองใหม่

### ขั้นตอนที่ 9: เริ่มการวัดค่าใหม่
```cpp
delay(1200);
sen66.startContinuousMeasurement();
delay(1200);
```
- รอให้ device พร้อม แล้วเริ่มการวัดค่าต่อเนื่องใหม่
- รอให้ระบบเสถียร

### ขั้นตอนที่ 10: คืนค่าจอแสดงผลและปลดล็อค
```cpp
if (originalDisplayBrightness >= 0) {
    oledDisplay.setBrightness(originalDisplayBrightness);
    originalDisplayBrightness = -1;
}
sen66PollingSuspended = false;
sen66ExclusiveI2c = false;
```
- เปิดจอแสดงผลด้วยค่า brightness เดิม
- ปลดล็อค I2C ให้ process อื่นใช้งานได้
- เปิดการอ่านค่า sensor ปกติ

## การจัดการ Error
### Error ที่จุดต่างๆ
1. **SEN66 ไม่พร้อม** → คืนค่าจอแสดงผลและจบการทำงาน
2. **หยุดการวัดค่าล้มเหลว** → คืนค่าจอแสดงผลและจบการทำงาน
3. **Calibration ล้มเหลว** → ลองใหม่สูงสุด 3 ครั้ง แต่ยังคงดำเนินการต่อ
4. **Device reset ล้มเหลว** → ลองใหม่สูงสุด 3 ครั้ง แต่ยังคงดำเนินการต่อ

### การคืนค่าจอแสดงผล
- ทุกจุดที่มี `return` จะมีการคืนค่าจอแสดงผลก่อนออกจากฟังก์ชัน
- ป้องกันไม่ให้จอแสดงผลดับค้างไว้

## ผลลัพธ์ที่ได้
### ข้อดี
1. **ลดการรบกวน I2C** - ปิดจอ OLED ระหว่าง calibration
2. **เพิ่มความเสถียรของ I2C Bus** - มีการ reset I2C bus
3. **เพิ่มความน่าเชื่อถือ** - มี retry mechanism
4. **UX ที่ดีขึ้น** - จอแสดงผลกลับมาทำงานปกติหลัง calibration
5. **การ debug ที่ดีขึ้น** - มี log ที่ชัดเจนในแต่ละขั้นตอน

### เวลาที่ใช้ทั้งหมด
- **กรณีสำเร็จในครั้งแรก**: ประมาณ 13-15 วินาที
- **กรณีต้อง retry**: อาจใช้เวลาถึง 25-30 วินาที (ขึ้นกับจำนวนครั้งที่ retry)

## การใช้งาน
```cpp
calibrateCO2Sensor(); // เรียกใช้ผ่านฟังก์ชันนี้
```

## หมายเหตุ
- การแก้ไขนี้เข้ากันได้กับโค้ดเดิมโดยไม่ต้องเปลี่ยนการเรียกใช้
- ไม่กระทับต่อฟีเจอร์อื่นๆ ของระบบ
- สามารถใช้งานได้กับทั้ง AirGradient ONE และ Open Air models