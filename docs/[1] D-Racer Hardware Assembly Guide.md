# D-Racer 조립 가이드
본 가이드는 D-Racer 부품 조립 안내서입니다. 구성은 아래와 같습니다.  
- 부품 리스트
- 조립 가이드
- 유의 사항

<br>

## 1 ) 부품 리스트
D-Racer 키트를 구성하는 부품 리스트와 실물 이미지는 Table 1, Figure 1과 같습니다.  

<p align="center">
  <b>Table 1. Components List</b>
</p>

<table align="center">
  <thead>
    <tr>
      <th>No.</th>
      <th>Component</th>
      <th>No.</th>
      <th>Component</th>
    </tr>
  </thead>
  <tbody>
    <tr><td align="center">1</td><td>D3-G (8GB/32GB)</td><td align="center">12</td><td>WiFi Dongle</td></tr>
    <tr><td align="center">2</td><td>Battery Module Board (*Waveshare)</td><td align="center">13</td><td>USB Camera</td></tr>
    <tr><td align="center">3</td><td>D3-G Board Plate</td><td align="center">14</td><td>Camera Support Bolt</td></tr>
    <tr><td align="center">4</td><td>I2C Interface Board</td><td align="center">15</td><td>Jumper Wire (Female-Female)</td></tr>
    <tr><td align="center">5</td><td>Front Support</td><td align="center">16</td><td>M3.0 Bolt (Length: 60mm)</td></tr>
    <tr><td align="center">6</td><td>USB-Hub Box</td><td align="center">17</td><td>M2.5 Bolt (Length: 55mm)</td></tr>
    <tr><td align="center">7</td><td>Rear Support</td><td align="center">18</td><td>M2.5 Bolt (Length: 25mm)</td></tr>
    <tr><td align="center">8</td><td>Nylon Support (*Waveshare)</td><td align="center">19</td><td>M2.5 Bolt (Length: 3mm)</td></tr>
    <tr><td align="center">9</td><td>Power Adaptor (*Waveshare)</td><td align="center">20</td><td>18650 Battery</td></tr>
    <tr><td align="center">10</td><td>Power Plug</td><td align="center">21</td><td>Vehicle Chassis (*Waveshare)</td></tr>
    <tr><td align="center">11</td><td>USB-Hub</td><td align="center">22</td><td>Joystick (*Waveshare)</td></tr>
  </tbody>
</table>

<br>

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure1-kit-components.png" alt="Kit Components">
  <br>
  <b>Figure 1. Kit Components</b>
</p>


D-Racer의 메인 보드인 D3-G의 사양(Specification)은 아래 table 2와 같습니다.  

<p align="center">
  <b>Table 2. D3-G Specifications</b>
</p>

<table align="center">
  <thead>
    <tr>
      <th colspan="2">Function</th>
      <th>Specification</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="5" align="center">SoC<br>(TCC8051)</td>
      <td align="center">Main Core</td>
      <td align="center">Cortex-A72 Quad @1.69GHz, 31,840 DMIPS</td>
    </tr>
    <tr>
      <td align="center">Sub-Core</td>
      <td align="center">Cortex-A53 Quad @1.45GHz, 13,340 DMIPS</td>
    </tr>
    <tr>
      <td align="center">Total DMIPS</td>
      <td align="center">45,180 DMIPS</td>
    </tr>
    <tr>
      <td align="center">MCU Core</td>
      <td align="center">Cortex-R5f @ 600MHz</td>
    </tr>
    <tr>
      <td align="center">GPU</td>
      <td align="center">PowerVR 9XTP GT9524, 168GFLOPS, OpenGL ES 3.0</td>
    </tr>
    <tr>
      <td align="center">RAM</td>
      <td align="center">LPDDR4X</td>
      <td align="center"><b>Option(1)</b> LPDDR4x 4GB, <b>Option(2)</b> LPDDR4x 8GB</td>
    </tr>
    <tr>
      <td rowspan="3" align="center">Storage</td>
      <td align="center">SNOR(NC)</td>
      <td align="center">Quad SPI,100MHz / 4MB (Automotive Boot Mode)</td>
    </tr>
    <tr>
      <td align="center">eMMC</td>
      <td align="center">MLC 32GB</td>
    </tr>
    <tr>
      <td align="center">Micro SD Card</td>
      <td align="center">Micro SD Card Socket</td>
    </tr>
    <tr>
      <td align="center">Display</td>
      <td align="center">DP</td>
      <td align="center">DP 1.4, 4-Lane (8.1 Gbps/lane)<br>Up to 4-Display - DP MST(Multi Stream Transport)</td>
    </tr>
    <tr>
      <td align="center">Camera</td>
      <td align="center">MIPI-CSI</td>
      <td align="center">MIPI CSI 2-Lane x2 (15pin)<br>Option: MIPI CSI 4-Lane by connector swap</td>
    </tr>
    <tr>
      <td colspan="2" align="center">USB</td>
      <td align="center">USB 3.0 Host (Type A), USB 2.0 Host(Type A), USB 2.0 Device(Type C)</td>
    </tr>
    <tr>
      <td colspan="2" align="center">PCIe</td>
      <td align="center">1 x PCIe 3.0 (1-Lane)</td>
    </tr>
    <tr>
      <td colspan="2" align="center">Ethernet</td>
      <td align="center">1 Gbps Legacy Ethernet</td>
    </tr>
    <tr>
      <td colspan="2" align="center">General Function Interface</td>
      <td align="center">2.54 mm pitch 40 pin (2x20) Header<br>(I2C, SPI, UART, I2S, MI2S, PWM, GPIOs)</td>
    </tr>
    <tr>
      <td colspan="2" align="center">CAN</td>
      <td align="center">2.54 mm pitch 10pin (2x5) Header,<br>3ch CAN w/ Transceiver Sub Board</td>
    </tr>
    <tr>
      <td colspan="2" align="center">Debug</td>
      <td align="center">2.54 mm pitch 8Pin (1x8) Header<br>Cortex UART Debug x 3ch</td>
    </tr>
    <tr>
      <td colspan="2" align="center">Switches</td>
      <td align="center">Tact switch for Reset,<br>Tact switch for Boot mode</td>
    </tr>
    <tr>
      <td colspan="2" align="center">Power</td>
      <td align="center">(Recommendation) 5V@5A</td>
    </tr>
    <tr>
      <td colspan="2" align="center">PCB</td>
      <td align="center">90mm * 120mm</td>
    </tr>
  </tbody>
</table>

<br>

## 2 ) 조립 가이드
D-Racer의 부품별 조립 가이드입니다.   
<br>


아래 순서대로 D-Racer 키트를 조립합니다.  


1. 전원 모듈 보드에 제공된 `배터리(4EA)`를 장착합니다(Figure 2).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure2-battery-connection.png" alt="Battery Connection">
      <br>
      <b>Figure 2. Battery Connection</b>
    </p>

    <br>

2. 전원 모듈 보드와 제공된 섀시를 아래 순서로 연결합니다.  

    **2.1** ESC 전원 핀과 보드 전원 소켓을 연결합니다(Figure 3).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure3-esc-power-connection.png" alt="ESC Power Connection">
      <br>
      <b>Figure 3. ESC Power Connection</b>
    </p>

    <br>

    **2.2** 전원 모듈 보드의 Servo 인터페이스에 맞춰 섀시 와이어를 결선합니다(Figure 4).  
    (+) - Red / (-) - Black / (s) - White   

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure4-servo-motor-connection.png" alt="Servo Motor Connection">
      <br>
      <b>Figure 4. Servo Motor Connection</b>
    </p>

    <br>

    **2.3** 전원 모듈 보드의 Motor 인터페이스에 맞춰 섀시 와이어를 결선합니다(Figure 5).  
    (+) - Red / (-) - Black / (s) - White   

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure5-dc-motor-connection.png" alt="DC Motor Connection">
      <br>
      <b>Figure 5. DC Motor Connection</b>
    </p>

    <br>

3. 전원 모듈 보드와 USB-Hub-Box를 장착합니다. 사용되는 볼트는 `M2.5 25mm(수량:2)`입니다(Figure 6).   

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure6-usb-hub-box-connection.png" alt="USB Hub Box Connection">
      <br>
      <b>Figure 6. USB Hub Box Connection</b>
    </p>

    <br>  

4. D-Racer-board-plate와 front-support, rear-support를 결착합니다. 사용되는 볼트는 `M2.5 55mm(수량:2)`, `M3.0 60mm(수량:2)`입니다. front-support 홀에는 `M2.5 볼트`를, rear-support 홀에는 `M3.0 볼트`를 결착합니다(Figure 7).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure7-supports-connection.png" alt="Supports Connection">
      <br>
      <b>Figure 7. Supports Connection</b>
    </p>

    <br>


5. 해당 나사와 부품(표시된 부분)을 드라이버를 통해 제거해주세요. (Figure 8)  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure8-bolts-removal.png" alt="Bolts Removal">
      <br>
      <b>Figure 8. Bolts Removal</b>
    </p>

    <br>




6. 제공된 `나일론 지지대(수량:4)`를 이용해 (1) 전원 모듈 보드와 (2) D-Racer-board-plate를 결합합니다(Figure 9).  
    **이때 볼트를 너무 강하게 조이면 3D 플레이트가 파손될 수 있으므로, 적정 힘으로 장착해 주세요.**  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure9-chasis-assembly.png" alt="Chassis Assembly">
      <br>
      <b>Figure 9. Chassis Assembly</b>
    </p>

    <br>

7. D-Racer-board-plate 위에 D3-G를 장착합니다. 사용되는 볼트는 `M2.5 3mm(수량:2)`입니다(Figure 10).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure10-mounting-d3-g.png" alt="Mounting D3-G">
      <br>
      <b>Figure 10. Mounting D3-G</b>
    </p>

    <br>

8. 카메라를 D-Racer-board-plate 앞면에 위치시키고, `카메라 전용 나사(수량:1)`를 이용해 고정합니다(Figure 11). 

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure11-mounting-camera.png" alt="Mounting Camera">
      <br>
      <b>Figure 11. Mounting Camera</b>
    </p>

    <br>

9. 카메라 USB 커넥터를 아래와 같이 부착합니다(Figure 12).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure12-camera-usb-connection.png" alt="Camera USB Connection">
      <br>
      <b>Figure 12. Camera USB Connection</b>
    </p>

    <br>

10. D-Racer I2C 인터페이스 보드에 `점퍼선(Female/Female, 수량:4)`을 연결합니다(Figure 13).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure13-i2c-interface-board.png" alt="I2C Interface Board">
      <br>
      <b>Figure 13. I2C Interface Board</b>
    </p>

    <br>

11. 제공된 D-Racer I2C 인터페이스 박스를 섀시 위 핀헤더에 연결합니다(Figure 14).  
    **이때 반드시 6개의 핀헤더가 모두 장착되어야 하므로, 옆에서 확인하면서 장착해 주세요.**  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure14-i2c-interface-board-connection.png" alt="I2C Interface Board Connection">
      <br>
      <b>Figure 14. I2C Interface Board Connection</b>
    </p>

    <br>

12. D3-G GPIO 핀맵에 대응하여 점퍼선을 연결합니다. 이때 반드시 I2C 인터페이스 보드의 3.3V, SDA, SCL, GND 라인을 보드의 GPIO 핀맵에 맞춰서 연결합니다. (Figure 15)  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure15-i2c-wire-connection.png" alt="I2C Wire Connection">
      <br>
      <b>Figure 15. I2C Wire Connection</b>
    </p>

    <br>

13. 배터리 충전용 케이블을 섀시 전원부에 연결한 뒤, 어댑터에 전원을 인가해 배터리를 충전합니다(Figure 16). 완충시,초록색으로 어댑터에 출력됩니다.    

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure16-charging-battery.png" alt="Charging Battery">
      <br>
      <b>Figure 16. Charging Battery</b>
    </p>

    <br> 

14. 제공된 USB-Hub를 D-Racer 옆면의 USB-Hub-Box에 장착하고, USB 커넥터는 D3-G 보드 USB 소켓에 연결합니다(Figure 17).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure17-usb-hub-connection.png" alt="USB Hub Connection">
      <br>
      <b>Figure 17. USB Hub Connection</b>
    </p>

    <br>

15. 제공된 와이파이 동글 장치를 USB-Hub 앞쪽 소켓에 장착합니다. (Figure 18)  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure18-wifi-dongle-connection.png" alt="WiFi Dongle Connection">
      <br>
      <b>Figure 18. WiFi Dongle Connection</b>
    </p>

    <br>

16. 제공된 조이스틱에 `AA배터리(수량:2)`를 넣고, 리시버는 USB-Hub에 장착합니다(Figure 19).  
    조이스틱은 뒷편 `ON/OFF 스위치`로 제어할 수 있습니다.    

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure19-joystick-connection.png" alt="Joystick Connection">
      <br>
      <b>Figure 19. Joystick Connection</b>
    </p>

    <br>

17. 제공된 전원 플러그를 양쪽 소켓에 연결합니다.(Figure 20)  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure20-power-plug-connection.png" alt="Power Plug Connection">
      <br>
      <b>Figure 20. Power Plug Connection</b>
    </p>

    <br>


18. 전원 모듈 보드 전원 스위치를 ON으로 켜고, 보드의 전원 LED 점등 여부를 확인합니다. 전원이 들어오면 전체 조립이 완료된 것입니다(Figure 21).  

    <p align="center">
      <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/1/figure21-power-on-d-racer.png" alt="Power On D-Racer">
      <br>
      <b>Figure 21. Power On D-Racer</b>
    </p>

    <br>

## 3. 유의 사항
키트 조립 및 운영 간 유의 사항에 대해 안내합니다. 


1. `D3-G의 정격 전압은 5V/5A`입니다. 반드시 제공된 케이블(수/수 타입)로 보드 전원을 인가해 주세요. `배터리 전용 어댑터는 8.4V/2A`이므로 **D3-G에 직접 인가하면 안 됩니다**.

2. 제공된 배터리 외에 **다른 배터리 사용은 권장하지 않습니다**. 제공 배터리는 보호회로가 있는 제품으로, 과충전 방지 기능을 통해 안전사고 예방에 도움이 됩니다. 

3. 안전상 이유로 배터리 충전 중에는 장시간 자리를 비우지 마시기 바랍니다.

4. 배터리 충전 상태는 어댑터 LED`(Red: 충전 중 / Green: 충전 완료)` 또는 이후 구동할 Monitor 패키지에서 확인할 수 있습니다.

5. 조립 중 3D 출력물 파손에 유의해 주세요. 부품이 파손되면 운영기관에 문의해 주세요.

6. 대회 진행 중 카메라 렌즈 파손에 유의해 주세요.  
