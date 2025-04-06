# MISORAオペレーターPCで扱うGUIプログラム
## 読み込み時の書き換え
 - CMakeLists.txt
    - 2 line : 
    ~~~bash!
    project(misora_gui) -> project(misora_operator)
    ~~~ 
 - package.xml
    - 4 line : 
    ~~~bash! 
    <name>misora_gui</name> -> <name>misora_operator<name>
    ~~~
## 各ミッションごとに切り替え可能
 - 実行時にparameter: my_parameterをP1~4,P6に設定
~~~bash!
ros2 run misora_operator misora_gui --ros-args -p my_parameter:=P1
~~~
