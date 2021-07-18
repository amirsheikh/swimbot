import serial
serialport = serial.Serial ("/dev/ttyS0")    #Open named port
serialport.baudrate = 115200
MAX_Speed = 20
MIN_Speed = -20
def speed_maker(speed):
    if(speed >= MIN_Speed and speed <= MAX_Speed):
        if(speed >= 0):
            digit_10 = int((speed/10)%10)
            digit_1 = int(speed%10)
            digit_01 = int((speed*10)%10)
            result = "+" + str(digit_10) + str(digit_1) + str(digit_01)
        else:
            speed= -1*speed
            digit_10 = int((speed/10)%10)
            digit_1 = int(speed%10)
            digit_01 = int((speed*10)%10)
            result = "-" + str(digit_10) + str(digit_1) + str(digit_01)
            print(result)
    elif(speed < MIN_Speed) :
        result = speed_maker(MIN_Speed)
    else:
        result = speed_maker(MAX_Speed)
    return result


while True:
    MR_speed = float(input())
    MR_speed = speed_maker(MR_speed)
    ML_speed = float(input())
    ML_speed = speed_maker(ML_speed)
    data = MR_speed + ML_speed
    serialport.write(data.encode())
    print(data)



