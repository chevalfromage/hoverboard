import serial
import time
import matplotlib.pyplot as plt
import numpy as np

L_modifR=[0,0,50,50,50,50,50,50,50,50,50]
def construct_vel():
	for k in range(len(L_modifR)):
		velR.append(1550+L_modifR[k])
		velL.append(1550)
	velR.append(1550)
	velL.append(1550)

velR=[]
velL=[]
construct_vel()

velRi=0
s_velR=str(velR[0])
s_velL=str(velL[0])

ser = serial.Serial(port= "COM10", baudrate=115200,timeout=1)#your COM
#ser.open()

if not ser.isOpen():
	print("Erreur d\'ouverture du port")
	exit(1)


commands=[0,0,0,0]
commandR=[]
commandL=[]
vR=[]
vL=[]

#try:
envoieR = f"velR {s_velR}\n"
envoieL = f"velL {s_velL}\n"
ser.write(envoieR.encode())
ser.write(envoieL.encode())
#ser.write("log\n".encode())
precedent_time=time.time()
while (velRi<len(velR)-1):

	if(time.time()-precedent_time>1):
		precedent_time=time.time()
		velRi+=1
		s_velR=str(velR[velRi])
		s_velL=str(velL[velRi])
		envoieR = f"velR {s_velR}\n"
		envoieL = f"velL {s_velL}\n"
		ser.write(envoieR.encode())
		ser.write(envoieL.encode())

	line = ser.readline()
	decoded_line=line.decode('ascii')

	commands=decoded_line[:-2].split(",")
	commandR.append(float(commands[2]))
	commandL.append(float(commands[3]))
	vR.append(float(commands[0]))
	vL.append(-float(commands[1]))


	time.sleep(0.001)

write_in_file = True
if write_in_file:
	fo = open('C:/Users/33782/Documents/ENSEIRB/stage 1a/python serial/output.csv', 'w')
	if not fo:
		print("Erreur d\'ouverture du fichier")
		exit(1)
	fo.write(f"vR,vL,commandeR,commandeL\n")
	for i in range(len(vR)):
		fo.write(f"{vR[i]},{vL[i]},{commandR[i]},{commandL[i]}\n")
	fo.close()


ser.close()


fig, (axe_command, axe_vitesse) = plt.subplots(2)

Ltemps = np.linspace(0,1,len(commandR))
axe_command.plot(Ltemps,commandR,label="commandR")
axe_command.plot(Ltemps,commandL,label="commandL")
axe_vitesse.plot(Ltemps,vR,label="vR")
axe_vitesse.plot(Ltemps,vL,label="vL")

axe_command.legend(loc="upper left")
axe_vitesse.legend(loc="upper left")
plt.tight_layout()
plt.show()

"""
except (KeyboardInterrupt, SystemExit):
	ser.write("log\n".encode())
	ser.write("silence\n".encode())
	ser.close()
	fo.close()
except:
	ser.write("log\n".encode())
	ser.write("silence\n".encode())
	print("Erreur interne")
	ser.close()
	fo.close()
"""