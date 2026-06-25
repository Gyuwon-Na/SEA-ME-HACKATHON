# D-Racer 개발환경 셋업 가이드
본 가이드는 D-Racer Hardware Assembly 이후에 진행하는 소프트웨어 개발환경 구성 안내서입니다.  
아래 내용을 포함합니다.
- 대회 공식 이미지 다운로드 
- D3-G 이미지 업로드
- D3-G TOPST 로그인
- 와이파이 설정 가이드
- VSCode Remote SSH 설정
- VSCode와 Vibe Coding Tool 연동(Codex, Claude)

사용자 PC의 권장 사양은 아래와 같습니다. 
- Windows 10/11
- RAM 8GB 이상 
- 저장장치 50GB 이상

<br>


## 1 ) D3-G Hackathon 공식 이미지 Firmware Download
1. 개인 PC에 대회 공식 이미지와 필요 유틸리티를 다운로드합니다.  
제공되는 D3-G 이미지는 `Ubuntu 22.04` 기반입니다.  
다운로드 URL은 아래와 같으며, 다운로드 받은 후 압축 파일을 해제합니다.  
[D-Racer Ubuntu Image URL < Click Here](https://topst-downloads.s3.ap-northeast-2.amazonaws.com/Ubuntu/22.04/D-Racer-ubuntu-22.04-v1.0.1.zip)


<br>

## 2 ) Firmware Upload to D3-G
1. VTC Driver(Windows, Ubuntu 호환)를 설치합니다.  
압축 해제한 파일에 진입하여, win10_64 내 설치 프로그램을 관리자 권한으로 설치합니다. Vendor Telechips Certification(VTC) 드라이버를 설치합니다.(Figure 1)  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure1-vtc-driver-installation.png" alt="VTC Driver Installation">
  <br>
  <b>Figure 1. VTC Driver Installation</b>
</p>

<br>


2. USB-C to A 케이블로 D3-G와 PC를 연결합니다. (Figure 2)  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure2-connect-usb-c-to-a-cable.png" alt="Connect USB-C to A Cable">
  <br>
  <b>Figure 2. Connect USB-C to A Cable</b>
</p>

<br>

3. BOOT 스위치를 누른채로 D3-G 보드에 전원 케이블을 연결합니다. (Figure 3)  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure3-d3-g-boot-mode.png" alt="D3-G Boot Mode">
  <br>
  <b>Figure 3. D3-G Boot Mode</b>
</p>

<br>

4. VTC Driver 연결 여부를 확인합니다. 
위와 같이 FWDN 모드에서 USB를 연결하면 Telechips VTC USB 드라이버가 Figure 4 와 같이 인식됩니다.  
**참고: VTC Driver는 V5.0.0.14 이상을 사용해야 합니다. 버전은 Windows 장치 관리자에서 확인할 수 있습니다.**  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure4-windows-device-manager.png" alt="Windows Device Manager">
  <br>
  <b>Figure 4. Windows Device Manager</b>
</p>

<br>

5. 압축 해제한 폴더 속 `fwdn.bat` 파일을 더블클릭하여 실행합니다.  
아래 Figure 5와 같이 진행되면 이미지 업로드가 정상적으로 완료된 것입니다.  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure5-running-fwdn.bat.png" alt="Running FWDN.bat">
  <br>
  <b>Figure 5. Running FWDN.bat</b>
</p>

<br>

## 3 ) D3-G 최초 로그인

1. UART 통신용 전용 케이블 드라이버를 설치합니다. 아래 경로를 통해서 PL2303_Prolific_v3.3.2.105.exe 파일을 다운로드 & 설치합니다. (Figure 6)  
[Prolific PL2303 Driver << Download Link](https://github.com/theAmberLion/Prolific/blob/main/PL2303_Prolific_v3.3.2.105.exe)

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure6-download-page-prolific-pl2303-driver.png" alt="Download Page Prolific PL2303 Driver">
  <br>
  <b>Figure 6. Download Page Prolific PL2303 Driver</b>
</p>

<br>

2. 터미널 에뮬레이터 MobaXTerm를 설치하고 실행합니다. 
압축 해제한 폴더에서 MobaXTerm를 설치합니다(Figure 7).
[MobaXTerm << Download Link](https://mobaxterm.mobatek.net/download.html)

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure7-mobaxterm-download-page.png" alt="MobaXTerm Download Page">
  <br>
  <b>Figure 7. MobaXTerm Download Page</b>
</p>

<br>

3. D3-G와 UART 전용 케이블을 아래 Figure 8와 같이 연결합니다.  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure8-connection-usb-to-ttl-cable.png" alt="Connection USB to TTL Cable">
  <br>
  <b>Figure 8. Connection USB to TTL Cable</b>
</p>

<br>

4. PL2303 드라이버 연결 확인을 진행합니다. 장치관리자의 포트(COM & LPT)에서 확인할 수 있습니다. 사용자마다 COM 의 숫자는 다를 수 있습니다.  
- 사용간, COM 번호가 조회되지 않는다면 아래 방향으로 진행해주세요.
  - PL2303 우클릭 후 드라이버 업데이트 클릭
  - 내 컴퓨터에서 드라이버 찾아보기
  - 컴퓨터의 사용 가능한 드라이버 목록에서 직접 선택 
  - 설치한 3.3.2.105 version 선택
  - 다음 선택 
  - 드라이버 설치 완료 확인 후 닫기

5. MobaXTerm에서 UART 접속을 진행합니다.   
5.1 Session에서 Serial을 선택합니다(Figure 10).  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure10-select-session.png" alt="Select Session">
  <br>
  <b>Figure 10. Select Session</b>
</p>

<br>
5.2 Basic Serial Setting에서 사용자 PC에 인식된 Serial Port를 선택하고, Speed는 115200으로 설정합니다. 인식된 시리얼 포트 번호는 Windows 장치 관리자에서 확인할 수 있습니다(Figure 11).    

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure11-basic-serial-setting.png" alt="Basic Serial Setting">
  <br>
  <b>Figure 11. Basic Serial Setting</b>
</p>

<br>
5.3 Advanced Serial Setting에서 Hardware Flow Control을 `None`으로 설정합니다.(Figure 12)  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure12-advanced-serial-setting.png" alt="Advanced Serial Setting">
  <br>
  <b>Figure 12. Advanced Serial Setting</b>
</p>

<br>
5.4 D3-G `Reset 스위치`를 눌러 재부팅하면 MobaXTerm에서 로그인할 수 있습니다.   
 Username: `topst` / Password: `topst`로 로그인합니다..  

<br>

## 4) D3-G 와이파이 셋업 
본 과정은 topst 계정으로 진입한 mobaXTerm에서 진행합니다.  
1. 아래 명령어로 Netplan 설정을 진행합니다. 편집기는 `nano`를 사용합니다. (vi 사용 가능)
```bash
sudo nano /etc/netplan/99-default.yaml
```

2. 아래와 같이 수정하고 파일 저장을 진행합니다.  
 - `nano` 에서는 방향키로 커서를 이동할 수 있습니다. 
 - 저장은 `ctrl + o` 입니다.
 - 에디터 종료 버튼은 `ctrl + x` 입니다. 
 - **수정 시 `wifis`와 `ethernets`를 같은 들여쓰기 레벨로 맞춰 주세요.**
```bash
topst@TOPST:~$ sudo nano /etc/netplan/99-default.yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:                    # << 여기서부터 새로 입력
    wlan0:
      optional: true
      access-points:
        "사용자 와이파이 이름":
          password: "와이파이 비밀번호"
      dhcp4: true
```
저장 후 `nano`를 종료합니다.

3. 저장된 파일을 확인하고 netplan 설정을 적용합니다.
```bash
cat /etc/netplan/99-default.yaml # 수정 잘 되었는지 확인

sudo netplan apply
```

4. D3-G의 무선 네트워크 주소를 확인합니다. 적용까지 약 1분 정도 소요됩니다.
```bash
ip addr

topst@TOPST:~$ ip addr
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host
       valid_lft forever preferred_lft forever
2: eth0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN group default qlen 1000
    link/ether xx:xx:xx:xx:xx:xx brd ff:ff:ff:ff:ff:ff
3: sit0@NONE: <NOARP> mtu 1480 qdisc noop state DOWN group default qlen 1000
    link/sit 0.0.0.0 brd 0.0.0.0
4: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether xx:xx:xx:xx:xx:xx brd ff:ff:ff:ff:ff:ff
    inet D3-G 무선랜 주소/24 brd 172.30.x.255 scope global dynamic noprefixroute wlan0
       valid_lft 2488sec preferred_lft 2488sec
    inet6 fe80::xxxx:xxxx:xxxx:xxxx/64 scope link
       valid_lft forever preferred_lft forever

```

<br>

## 5 ) D3-G 우분투 파티션 확장
D3-G 내 더 큰 용량의 eMMC로 사용하기 위해선 parted 를 통해 반드시 확장 작업이 필요합니다.  
본 과정은 topst 계정으로 진입한 mobaXTerm에서 진행합니다.  
아래 순으로 작업하여 확장하시기 바랍니다.   
```bash
su - # root 계정으로 전환 (비밀번호 : root)

parted
rescue 
Fix
Start: 0 # 0 입력
End : 100% # 100% 입력

resizepart 4
Yes 
100% # 이후 Ctrl + C로 parted 종료

# 이후 시스템 재부팅

sudo resize2fs /dev/mmcblk0p4 # sudo 추가
df -h # 확장된 저장용량 확인

# 시스템 재부팅 
df -h # 확장된 용량 확인
```
<br>


## 6 ) VSCode SSH Remote 셋업
D3-G 와 원격으로 통신하기 위한 가이드입니다.  
본 대회에서는 코드 편집과 D3-G 터미널 사용을 원격으로 진행하기 위해 VSCode 사용을 권장합니다.

1. 사용자 PC에 VSCode를 설치합니다.(Windows 10,11 설치)  
[VSCode << Download Link](https://code.visualstudio.com/Download)

2. VSCode를 실행하여 SSH 환경 설정을 진행합니다.   
- `Ctrl + Shift + P`로 명령 팔레트를 실행합니다.
- `Remote-SSH: Open SSH Configuration File`을 선택합니다. 
- `C:\Users\사용자\.ssh\config` 파일에 아래 내용을 입력하고 저장합니다.
- 이때 D3-G에서 확인한 무선 LAN 주소를 입력합니다. 
```bash
Host d-racer
  HostName 무선랜 주소
  User topst
```
3. 다시 명령 팔레트(`Ctrl + Shift + P`)에서 `Remote-SSH: Connect to Host`를 실행해 저장한 호스트를 선택합니다.
4. 새 창이 열리면 D3-G 비밀번호(`topst`)를 입력합니다.
5. 초기 원격 설정이 완료된 뒤 아래 Figure 13와 같이 열리면 성공입니다. 이제 원격으로 D3-G를 제어할 수 있습니다.  

<p align="center">
  <img src="https://raw.githubusercontent.com/topst-development/D-Racer-Kit/refs/heads/docs/revision/docs/asset/2/figure13-vscode-ssh-remote.png" alt="VSCode SSH Remote">
  <br>
  <b>Figure 13. VSCode SSH Remote</b>
</p>

<br>

## 7 ) 문제 해결 - VSCode Remote SSH 접속 실패

VSCode에서 Remote SSH 접속이 되지 않는 경우 아래 항목을 먼저 확인합니다.

1. D3-G에 Wi-Fi dongle이 연결되어 있는지 확인합니다.
2. 사용자 PC와 D3-G가 같은 네트워크 대역에 연결되어 있는지 확인합니다.
3. D3-G의 무선 LAN IP 주소가 VSCode SSH config의 `HostName`과 일치하는지 확인합니다.

위 항목을 확인했는데도 접속되지 않는 경우, 기존 SSH 접속 정보가 충돌했을 수 있습니다.

1. Windows 사용자 계정의 `.ssh` 폴더로 이동합니다.
   - 예: `C:\Users\사용자명\.ssh`
2. `known_hosts` 파일을 메모장으로 엽니다.
3. D3-G의 IP 주소와 관련된 줄을 삭제한 뒤 저장합니다.
4. VSCode에서 `Remote-SSH: Connect to Host`를 다시 실행합니다.
5. 접속 확인 메시지가 표시되면 승인하고, 비밀번호 `topst`를 입력합니다.