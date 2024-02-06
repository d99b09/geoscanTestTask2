# Тестовое задание Geoscan

+ Установка requirements

`pip install -r requirements.txt`

## Тестирование реализованных функций 

+ Запуск скрипта `FakeDrone.py`

`python FakeDrone.py`

+ Если необходимо задать другой порт, например 8080

`python FakeDrone.py 8080`

### Реализованные функции 

+ Чтение и отправка heartbeat
+ connected
```python
from pioneer_sdk import Pioneer
import time
pioneer_mini = Pioneer(ip='localhost', mavlink_port=8000)
for _ in range(10):
    print(pioneer_mini.connected())
    time.sleep(1)

pioneer_mini.close_connection()

del pioneer_mini
```
+ get_battery_status
```python
from pioneer_sdk import Pioneer
import time
pioneer_mini = Pioneer(ip='localhost', mavlink_port=8000)
for _ in range(10):
    print(pioneer_mini.get_battery_status())
    time.sleep(1)

pioneer_mini.close_connection()

del pioneer_mini
```

+ get_dist_sensor_data

```python
from pioneer_sdk import Pioneer
import time
pioneer_mini = Pioneer(ip='localhost', mavlink_port=8000)
for _ in range(10):
    print(pioneer_mini.get_dist_sensor_data())
    time.sleep(1)

pioneer_mini.close_connection()

del pioneer_mini

```

Также была попытка реализовать функцию arm()

```python
from pioneer_sdk import Pioneer
pioneer_mini = Pioneer(ip='localhost', mavlink_port=8000)

pioneer_mini.arm()

pioneer_mini.close_connection()

del pioneer_mini
```

Для работы этой функции в файле piosdk необходимо закоментировать 

``` python
elif msg.get_type() == 'COMMAND_ACK':
    msg._type += f'_{msg.command}'
    # if msg.command == 400 and msg.result_param2 is not None:
    #     self._preflight_state.update(BatteryLow=msg.result_param2 & 0b00000001)
    #     self._preflight_state.update(NavSystem=msg.result_param2 & 0b00000010)
    #     self._preflight_state.update(Area=msg.result_param2 & 0b00000100)
    #     self._preflight_state.update(Attitude=msg.result_param2 & 0b00001000)
    #     self._preflight_state.update(RcExpected=msg.result_param2 & 0b00010000)
    #     self._preflight_state.update(RcMode=msg.result_param2 & 0b00100000)
    #     self._preflight_state.update(RcUnexpected=msg.result_param2 & 0b01000000)
    #     self._preflight_state.update(UavStartAllowed=msg.result_param2 & 0b10000000)
```

