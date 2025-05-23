# Мобильный робот 

## Оглавление 

1. [Описание проекта](#Описание-проекта)
2. [Список комплектующие](#Список-комплектующие)
3. [Кинематика мобильного робота](#Кинематика-мобильного-робота)
4. [Электрическая схема подключения](#Электрическая-схема-подключения)
5. [Прошивка микроконтроллер](#Прошивка-микроконтроллер)

## Описание проекта

Данный робот предназначен для обучения по робототехники в различных учебных завадениях. Конструктивная основа имеет сетку универсального крепления, что позволит в дальнейшем дополнить робот другими элементамии. Ниже приведен список комплектющих, предназначенный для решения SLAM-задачи.

## Список комплектующие 

В папке print_model расположены STL-файлы с деталями, которые необходимо распечатать на FDM 3D принтере. Для сборки потребуется:

- микроконтроллер ESP32;
- бортовой пк raspberry 4 4gb;
- инерционно измерительный датчик IMU CMP10A;
- лидар RPLIDAR A1;
- два электропривода JGB37-250;
- DC-DC преобразователь для электроприводов с 36В на 7.5В LM2576HVS;
- DC-DC преобразователь для логики с 36В на 5В XL4016;
- аккумулятор 36В, 4.4 А/ч, 10S2P;
- драйвер для электроприводов L298N;

## Кинематика мобильного робота

Для данного проекта будет рассмотрена кинематическая модель мобильного робота, имеющего два ведущих колеса и движущегося в плоскости [1]. Поскольку робот движется в горизонтальной плоскости, то достаточно рассмотреть плоский случай. Ниже изображена соответствующая система координат.

![image](https://github.com/user-attachments/assets/68fc44cf-5fea-4a64-9d92-47c6bfcc4d4f)

Введены следующие обозначения:
-	$X$ и $Y$ – глобальные координаты;
-	$X_B$ и $Y_B$  – координаты, связанные с центром робота;
-	$ф$ –  угол ориентации робота относительно глобальных координат;
-	$r$ – радиус колес;
-	$b$ – ширина робота;
-	$ICR$ – мгновенный центр вращения;
-	$R$ – радиус поворота мобильного робота вокруг $ICR$;
-	$v_L$  и $v_R$ – скорость контакта левого и правого колеса;
-	$ω$ – угловая скорость относительно $ICR$;
- $V$ – линейная скорость робота.

Следуя из определения угловой скорости, ее можно выразить из следующих соотношений:

$$ ω*(R + b/2) = v_R $$
$$ ω*(R - b/2) = v_L $$

Из этих соотношений можно выразить угловую скорость и радиус поворота:

$$ ω = (v_R - v_L)*r/b $$
$$ R = b/2 * (v_R + v_L)/(v_R - v_L) $$

Учитывая вышенаписанные уравнения, линейная скорость мобильного робота определяется следующим образом:

$$ V = ω*R = (v_R + v_L)*r/2 $$

Таким образом система уравнений, описывающая кинематику мобильного робота, выглядит следующим образом:

$$ dx/dt = V * cos(ф) $$
$$ dy/dt = V * sin(ф)$$
$$ dф/dt =ω$$

## Электрическая схема подключения
![эл_схема](https://github.com/user-attachments/assets/c9ce414d-4dcb-4679-97c5-8a6bffce30ca)

## Прошивка микроконтроллера
В папке my-robot-2025_v2 лежит прошивка для микроконтроллера
