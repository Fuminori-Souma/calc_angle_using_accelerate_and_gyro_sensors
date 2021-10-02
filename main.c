#include <Wire.h>
#include <MsTimer2.h>

// MPU-6050�̃A�h���X�A���W�X�^�ݒ�l
#define MPU6050_WHO_AM_I     0x75  // Read Only
#define MPU6050_PWR_MGMT_1   0x6B  // Read and Write
#define MPU_ADDRESS  0x68

#define CTRL_PERIOD 50 // �������[ms]
#define PULSE_PER_REV 2000 // �G���R�[�_��1��]������̃p���X

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


// �f�o�C�X���������Ɏ��s�����
void setup()
{
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  //interupt���Z�b�g�A�b�v����
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);

  // ����̓ǂݏo��
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);  //MPU6050_PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();

  // ���샂�[�h�̓ǂݏo��
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);  //MPU6050_PWR_MGMT_1���W�X�^�̐ݒ�
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
    // �Z���T�Î~���̊e�����x�̃I�t�Z�b�g�l���Z�o
    for (i = 0; i < 10; i++)
    {
      calc_sensors_val();
      gyrosum += gyro_x;
    }
    gyrofst = gyrosum / 10;

    // �������t���O���I�t�ɂ���
    initflg = 0;
  }
  else
  {
    calc_sensors_val();

    // arcsin�͈̔͂𐧌�
    if ((acc_y * 90) > 90)
    {
      acc_y = 1;
    }
    else if ((acc_y * 90) < -90)
    {
      acc_y = -1;
    }

    // �����g����`��deg�ɕϊ�
    aang = asin(acc_y) * 90 / (PI / 2);

    // �����x�Z���T�̒l����A������Ԃ̊p�x���v�Z�i�P�̂ł͏����p�x��������Ȃ��G���R�[�_�ƃW���C���Ɏg�p�j
    if (!(angofst))
    {
      angofst = aang;
    }

    // ���݂̊p���x[deg/s]���Z�o
    curganv = gyro_x - gyrofst;

    // �����΍����J�b�g
    if (abs(curganv - preganv) < 1)
    {
      // �O��l�Ƃ̍������͂��ȏꍇ�́A�O��l�����ݒl�Ƃ��Čv�Z
      curganv = preganv;
    }
    else
    {
      // �p���x[deg/s]���`�ϕ����Ċp�x[deg]�ɕϊ�
      curgang += (preganv + curganv) * (conv.millsec2sec(CTRL_PERIOD) / 2.0);
    }

    Serial.print("En: ");
    Serial.print(counter * 360.0 / PULSE_PER_REV + angofst);
    temp = counter;

    Serial.print("\tAc: ");
    Serial.print(aang);

    Serial.print("\tGy: ");
    Serial.print(curgang + angofst);

    // �p���x�̐��l��ǉ�
    Serial.print("\tGy_raw: ");
    Serial.print(curganv);


    // �����Ń��C���̏����I���

    // �����������邽�߂�delay����
    curt = conv.microsec2sec(micros());
    diff = curt - pret;
    delay(CTRL_PERIOD - conv.sec2millsec(diff));

    // ���̎����̂��߂�diff�쐬����
    curt = conv.microsec2sec(micros());
    diff = curt - pret;

    // ����̒l��O��l�Ƃ��ĕێ�
    pret = curt;
    preganv = curganv;
    Serial.print("\n");
  }
}


void calc_sensors_val()
{
  // �������炪���C���̏���
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

  // �����x�l�𕪉�\�Ŋ����ĉ����x(G)�ɕϊ�����
  acc_x = axRaw / 16384.0;  //FS_SEL_0 16,384 LSB / g
  acc_y = ayRaw / 16384.0;
  acc_z = azRaw / 16384.0;

  // �p���x�l�𕪉�\�Ŋ����Ċp���x(degrees per sec)�ɕϊ�����
  gyro_x = gxRaw / 131.0;//FS_SEL_0 131 LSB / (��/s)
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