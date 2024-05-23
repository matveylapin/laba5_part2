# Модель 6-ти осевого робота манипулятора
## Описание
Проект предназначен для знакомства с возможностями SimScape Multibody. В качестве примера разработан робот манипулятор с 6 степенями свободы. Проведён расчёт прямой кинематики вручную для проверки реализации программы управления.

## Файлы проекта
```
.
├── Laba5_part2.prj
├── README.md
├── example_paths
│   └── example_path_1.m
├── examples
│   ├── actuator_example.slx
│   └── robot_path_follower.m
├── libs
│   ├── ForwardKinematics.m
│   └── InverseKinematics.m
├── models
│   ├── join_control_lite.slx
│   ├── join_lite.slx
│   ├── robot_main.slx
│   └── x6_dof_robot.slx
├── no_gtest
│   └── ForwardKinematics_test.m
├── scripts
│   ├── Deinitialization
│   │   ├── clear_trash.m
│   │   └── unload_workspace.m
│   ├── Initialization
│   │   └── load_constants.m
│   └── shortcuts
│       └── run_robot_example.m
└── visual
    ├── actuator_visual.slx
    └── xyz_probe.slx

```
### Описание директорий
* [example_paths](example_paths) - содержит примеры создания траектории для движения робота
* [examples](examples) - содержит примеры использования созданных моделей и библиотек, полезно для отладки проекта и демонстрации возможностей. Так же содержит пример работы модели робота [robot_main](examples/robot_main.slx).
* [libs](libs) - папка библиотек проекта, включает в себя написанные классы для удобной работы с роботом. Например [ForwardKinematics](libs/ForwardKinematics.m) решает прямую задачу кинематики между любыми СК робота.
* [models](models) - содержит разработанные для проекта simulink модели, включая основную модель робота.
* [no_gtest](no_gtest) - содерджит автотесты разработныных библиотек. Например [ForwardKinematics_test](no_gtest/ForwardKinematics_test.m) проверяет решение задачи прямой кинематики относительно расчитанных вручную уравнений. 
* [scripts](scripts) - скрипты запусаемы при открытии проектов, закрытии и вызовов shortcuts
* [visual](visual) - модели описывающие визуальное представление узлов робота.

## Работа с проектом
Для запуска примера движения по траектории можно вызвать shortcut run_robot_example, по умолчанию запускается траектория example_path_1. При вызове в воркспейс загружается траектория, расчитывается обратная кинематика, открывается окно с multibody моделью.
### Расчёт задачи обратной кинематики
TODO