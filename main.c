#include <Wire.h>
#include <MsTimer2.h>

// MPU-6050のアドレス、レジスタ設定値
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

#define CTRL_PERIOD 50 // 制御周期[ms]
#define PULSE_PER_REV 2000 // エンコーダの1回転当たりのパルス

volatile int temp, counter = 0;
double angofst = 0;
double curgang = 0;
double pregang = 0;
double preganv = 0;
boolean initflg = 1;
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

void calc_sensors_val(void);

class func_convert
{
  public:
    double sec2millsec(double input)
    { return input * 1000; }
    
    double millsec2sec(double input)
    { return input / 1000; }
    
    double microsec2sec(double input)
    { return input / 1000 / 1000; }
};


// デバイス初期化時に実行される
void setup()
{
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  //interuptをセットアップする
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);

  // 初回の読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // 動作モードの読み出し
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1レジスタの設定
  Wire.write(0x00);
  Wire.endTransmission();
}


void loop()
{
  static double curt, pret, diff, gyrofst;
  double aang, curganv;
  double gyrosum = 0;
  unsigned int ai;
  int i;

  func_convert conv;

  if (initflg)
  {
    // センサ静止時の各加速度のオフセット値を算出
    for (i = 0; i < 10; i++)
    {
      calc_sensors_val();
      gyrosum += gyro_x;
    }
    gyrofst = gyrosum / 10;

    // 初期化フラグをオフにする
    initflg = 0;
  }
  else
  {
    calc_sensors_val();

    // arcsinの範囲を制限
    if ((acc_y * 90) > 90)
    {
      acc_y = 1;
    }
    else if ((acc_y * 90) < -90)
    {
      acc_y = -1;
    }

    // 正弦波を線形のdegに変換
    aang = asin(acc_y) * 90 / (PI / 2);

    // 加速度センサの値から、初期状態の角度を計算（単体では初期角度が分からないエンコーダとジャイロに使用）
    if (!(angofst))
    {
      angofst = aang;
    }

    // 現在の角速度[deg/s]を算出
    curganv = gyro_x - gyrofst;

    // 微小偏差をカット
    if (abs(curganv - preganv) < 1)
    {
      // 前回値との差分が僅かな場合は、前回値を現在値として計算
      curganv = preganv;
    }
    else
    {
      // 角速度[deg/s]を台形積分して角度[deg]に変換
      curgang += (preganv + curganv) * (conv.millsec2sec(CTRL_PERIOD) / 2.0);
    }

    Serial.print("En: ");
    Serial.print(counter * 360.0 / PULSE_PER_REV + angofst);
    temp = counter;

    Serial.print("\tAc: ");
    Serial.print(aang);

    Serial.print("\tGy: ");
    Serial.print(curgang + angofst);

    // 角速度の生値を追加
    Serial.print("\tGy_raw: ");
    Serial.print(curganv);


    // ここでメインの処理終わり

    // 制御周期を作るためのdelay処理
    curt = conv.microsec2sec(micros());
    diff = curt - pret;
    delay(CTRL_PERIOD - conv.sec2millsec(diff));

    // 次の周期のためのdiff作成処理
    curt = conv.microsec2sec(micros());
    diff = curt - pret;

    // 今回の値を前回値として保持
    pret = curt;
    preganv = curganv;
    Serial.print("\n");
  }
}


void calc_sensors_val()
{
  // こっからがメインの処理
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  while (Wire.available() < 14);
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw, temp;

  axRaw = Wire.read() << 8 | Wire.read();
  ayRaw = Wire.read() << 8 | Wire.read();
  azRaw = Wire.read() << 8 | Wire.read();
  temp  = Wire.read() << 8 | Wire.read();
  gxRaw = Wire.read() << 8 | Wire.read();
  gyRaw = Wire.read() << 8 | Wire.read();
  gzRaw = Wire.read() << 8 | Wire.read();

  // 加速度値を分解能で割って加速度(G)に変換する
  acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  acc_y = ayRaw / 16384.0;
  acc_z = azRaw / 16384.0;

  // 角速度値を分解能で割って角速度(degrees per sec)に変換する
  gyro_x = gxRaw / 131.0;//FS_SEL_0 131 LSB / (°/s)
  gyro_y = gyRaw / 131.0;
  gyro_z = gzRaw / 131.0;
}

void ai0()
{
  if (digitalRead(3) == LOW)
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void ai1()
{
  if (digitalRead(2) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}