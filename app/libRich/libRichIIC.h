#ifndef IIC_H
#define IIC_H

/*
函数功能: 发送起始信号
当时钟线为高电平的时候，数据线由高电平变为低电平的过程
*/
void libRichIIC_SendStart();

/*
函数功能: 停止信号
当时钟线为高电平的时候，数据线由低电平变为高电平的过程
*/
void libRichIIC_SendStop();

/*
函数功能: 获取应答信号
返 回 值: 0表示获取到应答 1表示没有获取到应答
*/
unsigned char libRichIIC_GetAck();

/*
函数功能: 发送应答信号
函数参数：0表示应答 1表示非应答
*/
void libRichIIC_SendAck(unsigned char ack);

/*
函数功能: 发送一个字节数据
*/
void libRichIIC_SendOneByte(unsigned char dat);

/*
函数功能: 接收一个字节数据
*/
unsigned char libRichIIC_RecvOneByte(void);

#endif