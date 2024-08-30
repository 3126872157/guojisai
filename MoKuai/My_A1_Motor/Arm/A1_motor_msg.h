#ifndef A1_MOTOR_MSG_H
#define A1_MOTOR_MSG_H

#include "main.h"

#pragma pack(push, 1) // �ṹ�壬�������ڴ������1�ֽڶ���

// �����壬uint32_t�Ŀ���������洢
union Fp32
{
    uint32_t u;
    float f;
};

// �����õ����������ݽṹ
typedef struct
{
    uint8_t start[2]; // ��ͷ
    uint8_t motorID;  // ���ID  0,1,2,3 ...   0xBB ��ʾ�����е���㲥����ʱ�޷��أ�
    uint8_t reserved;
} COMHead;

// ���� �����������
typedef struct // �� 4���ֽ�һ������ ����Ȼ�����������
{
    uint8_t mode;      // �ؽ�ģʽѡ��
    uint8_t ModifyBit; // ������Ʋ����޸�λ
    uint8_t ReadBit;   // ������Ʋ�������λ
    uint8_t reserved;

    uint32_t Modify; // ��������޸� ������
    // ʵ�ʸ�FOC��ָ������Ϊ��K_P*delta_Pos + K_W*delta_W + T
    int16_t T;   // �����ؽڵ�������أ������������أ�x256, 		7 + 8 ����
    int16_t W;   // �����ؽ��ٶ� �����������ٶȣ� x128,       	8 + 7����
    int32_t Pos; // �����ؽ�λ�� x 16384/6.2832, 14λ������������0������������ؽڻ����Ա�����0��Ϊ׼��

    int16_t K_P; // �ؽڸն�ϵ�� x2048  4+11 ����
    int16_t K_W; // �ؽ��ٶ�ϵ�� x1024  5+10 ����

    uint8_t LowHzMotorCmdIndex; // �����Ƶ�ʿ������������, 0-7, �ֱ����LowHzMotorCmd�е�8���ֽ�
    uint8_t LowHzMotorCmdByte;  // �����Ƶ�ʿ���������ֽ�

    uint32_t Res; // ͨѶ �����ֽ�
} MasterComdV3;

// ���� ��������������ݰ�
typedef struct
{
    COMHead head;
    MasterComdV3 Mdata;
    uint32_t CRCdata;
} MasterComdDataV3;

// ���� ��������
typedef struct // �� 4���ֽ�һ������ ����Ȼ�����������
{
    uint8_t mode;    // ��ǰ�ؽ�ģʽ
    uint8_t ReadBit; // ������Ʋ����޸�     �Ƿ�ɹ�λ
    int8_t Temp;     // �����ǰƽ���¶�
    uint8_t MError;  // ������� ��ʶ

    int32_t Read; // ��ȡ�ĵ�ǰ ��� �Ŀ�������
    int16_t T;    // ��ǰʵ�ʵ���������       7 + 8 ����

    int16_t W; // ��ǰʵ�ʵ���ٶȣ����٣�   8 + 7 ����
    float LW;  // ��ǰʵ�ʵ���ٶȣ����٣�

    int16_t W2;  // ��ǰʵ�ʹؽ��ٶȣ����٣�   8 + 7 ����
    int32_t LW2; // ��ǰʵ�ʹؽ��ٶȣ����٣�

    int16_t Acc;    // ���ת�Ӽ��ٶ�       15+0 ����  ������С
    int16_t OutAcc; // �������ٶ�         12+3 ����  �����ϴ�

    int32_t Pos;  // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
    int32_t Pos2; // �ؽڱ�����λ��(���������)

    int16_t gyro[3]; // ���������6�ᴫ��������
    int16_t acc[3];

    // ��������������
    int16_t Fgyro[3]; //
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp; // 8λ��ʾ���¶�  7λ��-28~100�ȣ�  1λ0.5�ȷֱ���

    int16_t Force16; // ����������16λ����
    int8_t Force8;   // ����������8λ����

    uint8_t FError; //  ��˴����������ʶ

    int8_t Res; // ͨѶ �����ֽ�

} ServoComdV3; // �������ݰ��İ�ͷ ��CRC 78�ֽڣ�4+70+4��

// ���� ����������ݰ�
typedef struct
{
    COMHead head;
    ServoComdV3 Mdata;
    uint32_t CRCdata;
} ServoComdDataV3; // ��������

// ���� ���͸�ʽ������
typedef struct
{
    MasterComdDataV3 motor_send_data; // ����������ݽṹ�壨����ʱ�������
    // �����͵ĸ�������
    unsigned short id;   // ���ID��0xBB����ȫ�����
    unsigned short mode; // 0:����, 5:����ת��, 10:�ջ�FOC����
    // ʵ�ʸ�FOC��ָ������Ϊ��K_P*delta_Pos + K_W*delta_W + T
    float T;   // �����ؽڵ�������أ������������أ���Nm��
    float W;   // �����ؽ��ٶȣ����������ٶȣ�(rad/s)
    float Pos; // �����ؽ�λ�ã�rad��
    float K_P; // �ؽڸն�ϵ��
    float K_W; // �ؽ��ٶ�ϵ��

    uint32_t Res;
} motor_send_t;

// ���� ��ʽ����������
typedef struct
{
    ServoComdDataV3 motor_recv_data; // ����������ݰ�������ʱ�������
    // ���������ĸ�������
    unsigned char motor_id; // ���ID
    unsigned char mode;     // 0:����, 5:����ת��, 10:�ջ�FOC����
    int Temp;               // �¶�
    unsigned char MError;   // ������

    float T;   // ��ǰʵ�ʵ���������
    float W;   // ��ǰʵ�ʵ���ٶȣ����٣�
    float LW;  // ��ǰʵ�ʵ���ٶȣ����٣�
    float Acc; // ���ת�Ӽ��ٶ�
    float Pos; // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��

    float gyro[3]; // ���������6�ᴫ��������
    float acc[3];
} motor_recv_t;

#pragma pack(pop)

#endif
