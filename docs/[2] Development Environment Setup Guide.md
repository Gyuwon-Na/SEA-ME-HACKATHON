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
[D-Racer Ubuntu Image URL < Click Here](https://topst-downloads.s3.ap-northeast-2.amazonaws.com/Ubuntu/22.04/D-Racer-ubuntu-22.04-v1.0.0.zip)


<br>

## 2 ) Firmware Upload to D3-G
1. VTC Driver(Windows, Ubuntu 호환)를 설치합니다.  
압축 해제한 파일에 진입하여, 관리자 권한으로 Vendor Telechips Certification(VTC) 드라이버를 설치합니다.(Figure xx)  
<사진첨부-드라이버 설치 장면>

2. USB-C to A 케이블로 D3-G와 PC를 연결합니다. (Figure xx)  
<사진 첨부 - 차량 위 D3-G와 PC와 usb-c-to-a로 연결하는 모습>

3. BOOT 스위치를 누른채로 D3-G 보드에 전원 케이블을 연결합니다. (Figure xx)  
<사진 첨부 - boot 스위치 누른 모습과 케이블 연결 직전 모습>

4. VTC Driver 연결 여부를 확인합니다. 
위와 같이 FWDN 모드에서 USB를 연결하면 Telechips VTC USB 드라이버가 Figure xx 와 같이 인식됩니다.  
**참고: VTC Driver는 V5.0.0.14 이상을 사용해야 합니다. 버전은 Windows 장치 관리자에서 확인할 수 있습니다.**  
<사진 첨부 - Telechips VTC 드라이버 잡힌 모습 캡쳐>

5. 압축 해제한 폴더 속 `fwdn.bat` 파일을 더블클릭하여 실행합니다.  
아래 Figure xx와 같이 진행되면 이미지 업로드가 정상적으로 완료된 것입니다.  
<사진 첨부 - 이미지 다운로드 모습 캡쳐>

<br>

## 3 ) D3-G 최초 로그인

1. UART 통신용 전용 케이블 드라이버를 설치합니다. 
압축 해제한 폴더에서 CP210X 드라이버를 설치합니다(Figure xx).
<사진 첨부 -  CP210x 설치하는 모습>

2. 터미널 에뮬레이터 MobaXTerm를 설치하고 실행합니다. 
압축 해제한 폴더에서 MobaXTerm를 설치합니다(Figure xx).
<사진 첨부 -  MobaXterm 설치하는 모습>

3. D3-G와 UART 전용 케이블을 아래 Figure xx와 같이 연결합니다.  
<사진 첨부 - UART 케이블 연결 모습>

4. MobaXTerm에서 UART 접속을 진행합니다.   
4.1 Session에서 Serial을 선택합니다(Figure xx).  
<사진첨부 - MobaXTERM 에서 Session 선택 스크린샷>  
<br>
4.2 Basic Serial Setting에서 사용자 PC에 인식된 Serial Port를 선택하고, Speed는 115200으로 설정합니다. 인식된 시리얼 포트 번호는 Windows 장치 관리자에서 확인할 수 있습니다(Figure xx).    
<사진첨부 - Serial Port 와 Speed 고른 모습 + 옆 사이드에 윈도우-장치관리자 화면까지 같이 캡쳐>  
<br>
4.3 Advanced Serial Setting에서 Hardware Flow Control을 `None`으로 설정합니다.(Figure)  
<사진 첨부 - flow control none 캡쳐>  
<br>
4.4 D3-G `Reset 스위치`를 눌러 재부팅하면 MobaXTerm에서 로그인할 수 있습니다.   
 Username: `topst` / Password: `topst`로 로그인합니다(Figure xx).  
<사진 첨부 - 로그인한 모습 캡쳐>

<br>

## 4) D3-G 와이파이 셋업 
본 과정은 topst 계정으로 진입한 mobaXTerm에서 진행합니다.  
1. 아래 명령어로 Netplan 설정을 진행합니다. 편집기는 `vi`를 사용합니다.
```bash
sudo vi /etc/netplan/99-default.yaml
```

2. 아래와 같이 수정하고 파일 저장을 진행합니다.  
`vi` 입력 모드는 `(ESC 키 입력 > i 또는 a)`이며, 명령 모드에서는 `h`, `j`, `k`, `l`로 커서를 이동할 수 있습니다.  
**수정 시 `wifis`와 `ethernets`를 같은 들여쓰기 레벨로 맞춰 주세요.**
```bash
topst@TOPST:~$ vi  /etc/netplan/99-default.yaml
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
저장 후 `vi`를 종료합니다(ESC 모드 > `:` > `wq` 순으로 입력).
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
D3-G 내 더 큰 용량의 eMMC로 사용하기 위해선 parted 를 통해 확장 작업이 필요합니다.  
본 과정은 topst 계정으로 진입한 mobaXTerm에서 진행합니다.  
아래 순으로 작업하여 확장하시기 바랍니다.   
```bash
su - # root 계정으로 전환 (비밀번호 : root)

parted 
Fix
Start: 0 # 0 입력
End : 100% # 100% 입력

resizepart 4 

# 이후 Ctrl + C로 parted 종료
resize2fs /dev/mmcblk0p4 
df -h # 확장된 저장용량 확인

# 재부팅 후 df -h 다시 확인
```
<br>


## 6 ) VSCode SSH Remote 셋업
D3-G 와 원격으로 통신하기 위한 가이드입니다.  
본 대회에서는 코드 편집과 D3-G 터미널 사용을 원격으로 진행하기 위해 VSCode 사용을 권장합니다.

1. 사용자 PC에 VSCode를 설치합니다.  
<url 첨부 - vscode 설치 경로>

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
5. 초기 원격 설정이 완료된 뒤 아래 Figure xx와 같이 열리면 성공입니다. 이제 원격으로 D3-G를 제어할 수 있습니다.  
<사진 첨부 - 아무것도 열리지 않은 베이스 사진>

<br>

## 7 ) Vibe Coding Tool Setup

원격으로 D3-G에 접속한 VSCode에서 진행합니다.  
VSCode 왼쪽 사이드바의 Extensions(단축키: `Ctrl + Shift + X`)에서 Codex 및 Claude Extension을 D3-G에 설치할 수 있습니다(Figure xx).  
<사진 첨부 - Extension 열어서 codex & claude 캡쳐>
설치 후에는 사용자가 보유한 툴의 API 키 또는 계정을 등록하여 D3-G에서 AI 기능을 사용할 수 있습니다. 
