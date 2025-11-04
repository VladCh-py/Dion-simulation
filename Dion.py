import numpy as np
import matplotlib.pyplot as plt
import keyboard

'''
Данная программа позволяет проводить симуляцию движения диона в электрических и магнитном полях. рассчет производится методом рунге-кутта 4го порядка.
Электростатическое поле направлено по оси Х. Вектор магнитной индукции по оси Z. Возможно задать компоненты скорости диона, массу, эелктрический заряд и магнитный заряд.
для завершения программы нажать на "esс". Для управления анимацией нажать на "ctrl"
'''

# Характеристики поля
E_x = 0
B_z = 10

# Характеристики диона (скорости/ масса / заряд электрический/ заряд магнитный)
Velocity_X=1
Velocity_Y=0
Velocity_Z=0
Mass=1.0
Electric_charge=10
# Magnetic_charge=37.5

Magnetic_charge=0

# Параметры симуляции
# Максимальное число итераций
iteration_limit = 2500000
# устанавливает количество итераций перед каждым обновлением отображения
sim_pause_ter=100
# Временной шаг
dt = 0.00001
# Включение симуляции
animation=True
# Режим симуляции (значения "3D" и "2D")
animation_type="3D"

# раскомментировать если необходимо задавать значения вручную

# print("Введите параметры симуляции: ")
# print("Модуль электрического поля (направлено по Ох) Е_х= ", end='')
# E_x=float(input())
# print("Модуль вектора магнитной индукции (направлено по Oz) B_z= ", end='')
# B_z=float(input())
# print("Магнитный заряд = ", end='')
# Magnetic_charge=float(input())
# print("Электрический заряд = ", end='')
# Electric_charge=float(input())
# # print("")

# print("Составляющие вектора скорости")
# print("V_x= ", end='') 
# Velocity_X=float(input())
# print("V_y= ", end='')
# Velocity_y=float(input())
# print("V_z= ", end='')
# Velocity_z=float(input())
# print("Провести симуляцию в 3D/2D (Введите '3' или '2')? ")
# animation_type="2D"
# if str(input())=="3": animation_type="3D"


# print("Настроить параметры симуляции (да/нет)?")
# if str(input())=="да":
#     print("Максимум итераций ")
#     iteration_limit==int(input())
#     print("Частота обновления картинки (в итерациях) ")
#     sim_pause_ter==int(input())
#     print("временной шаг dt")
#     dt=float(input())



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Функция рисования траектории в 3D
def animation_3D(x, y, z):
    plt.pause(0.00001)
    ax.cla()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.plot(x, y, z)
    ax.scatter(x[-1], y[-1], z[-1], color="red")
# Функция рисования зависимости координат от времени
def animation_time(time, x, y, z, clear):

    plt.subplot(3, 1, 1)
    plt.plot(time, x)
    plt.scatter(time[-1], x[-1], color="red")
    plt.ylabel('X')
    plt.xlabel('t')
    plt.title('X(t)')
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(time, y)
    plt.scatter(time[-1], y[-1], color="red")
    plt.ylabel('Y')
    plt.xlabel('t')
    plt.title('Y(t)')
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(time, z)
    plt.scatter(time[-1], z[-1], color="red")
    plt.ylabel('Z')
    plt.xlabel('t')
    plt.title('Z(t)')
    plt.grid(True)
    plt.pause(0.1)
    if clear:plt.draw()# Если график закрывать не надо
    else: plt.show()
    plt.clf()

# Включени/выключение отображения в реальном времени
def ANIMATION(): 
    global animation
    animation= not animation
# Выключение симуляции
def STOP():
    global running
    running=False

keyboard.add_hotkey("esc", STOP)
keyboard.add_hotkey("ctrl", ANIMATION)

# Рассчет ускорения диона
def calculation_acceleration(velocity, mass, E_charge, B_charge):
    acceleration_x = (E_charge *E_x) / mass + (E_charge * velocity[1] * B_z) / mass
    acceleration_y = -(E_charge * velocity[0] * B_z) / mass - (B_charge * velocity[2] * E_x) / mass
    acceleration_z = (B_charge * B_z) / mass + (B_charge * velocity[1] * E_x) / mass
    return np.array([acceleration_x, acceleration_y, acceleration_z])

# Функция расчета обновления состояния диона
def update_DION(DION, dt):
    positions = []
    DION.runge_kutta(dt)
    positions.append(DION.position.copy())
    return np.array(positions)

# Задания диона как класса
class Dion:
    def __init__(self, mass, E_charge, B_charge, velocity_x,  velocity_y,  velocity_z):
        self.mass = float(mass)
        self.E_charge = float(E_charge)
        self.B_charge = float(B_charge)
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([float(velocity_x), float(velocity_y), float(velocity_z)])
# Функция расчета скоростей и координат методо Рунге-Кутта 4ого порядка
    def runge_kutta(self, dt):
        k1_v = calculation_acceleration(self.velocity, self.mass,self.E_charge,self.B_charge)
        k1_x = self.velocity

        k2_v = calculation_acceleration(self.velocity+0.5*dt*k1_v, self.mass,self.E_charge,self.B_charge)
        k2_x = self.velocity + 0.5 * dt * k1_x

        k3_v = calculation_acceleration(self.velocity+0.5*dt*k2_v, self.mass,self.E_charge,self.B_charge)
        k3_x = self.velocity + 0.5 * dt * k2_x

        k4_v = calculation_acceleration(self.velocity+dt*k3_v, self.mass,self.E_charge,self.B_charge)
        k4_x = self.velocity + dt * k3_x

        self.position += (dt / 6) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
        # self.position += self.velocity
        self.velocity += (dt / 6) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)

# Присваиваем диону заданные параметры
DION = Dion(mass=Mass, E_charge=Electric_charge, B_charge=Magnetic_charge, velocity_x=Velocity_X, velocity_y=Velocity_Y, velocity_z=Velocity_Z)

# Задание вспомогательных массивов и переменных
X=[]
Y=[]
Z=[]
Time=[]
running=True
iteration=0

now_sim_iter=0
# Основной цикл программы (идет либо пока не пройдет все итерации, либо пока не будет остановлен вручную)
while running and iteration<iteration_limit:
    positions = update_DION(DION, dt) # Обновление состояние диона
    X.append(DION.position[0]) # Запоминаем его координаты
    Y.append(DION.position[1])
    Z.append(DION.position[2])
    Time.append(iteration)
    
    if animation and sim_pause_ter<now_sim_iter: # Если нужно, отображаем анимацию
        if animation_type=="3D": 
            animation_3D(X, Y, Z) 
        else: animation_time(Time, X, Y, Z, clear=True)
        now_sim_iter=0
    
    now_sim_iter+=1
    iteration+=1
    print("Симуляция в процессе... Текущая итерация: "+str(iteration) +" Выполнено: "+f"{(iteration*100/iteration_limit):.3f}"+"%")
    
print("Симуляция завершена. Выполнено итераций: "+str(iteration)+" ("+f"{(iteration*100/iteration_limit):.1f}"+"%)")

# В конце выводим итоговую траекторию
if animation_type=="3D": animation_3D(X, Y, Z) 
else: animation_time(Time, X, Y, Z, clear=False)
plt.show()
plt.draw()


