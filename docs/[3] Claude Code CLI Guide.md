# Claude Code CLI 사용 가이드
본 가이드는 D-Racer 개발 환경에서 Claude Code CLI를 설치하고 사용하는 방법을 담은 문서입니다.
아래 내용을 포함합니다.
- Claude Code CLI 설치
- Claude Code 실행 및 로그인
- Claude Code 모델 및 추론 능력 설정
- 이전 세션 다시 불러오기
- 권한 확인 생략 모드 실행

<br>

## 1 ) Claude Code CLI 설치
Claude Code는 터미널에서 실행하는 AI 코딩 도구입니다.
D-Racer 작업 디렉터리에서 코드 수정, 파일 확인, 명령어 실행 등을 대화형으로 진행할 수 있습니다.

먼저 `curl`이 설치되어 있는지 확인합니다.

```bash
curl --version
```

`curl`이 설치되어 있지 않다면 아래 명령어로 설치합니다.

```bash
sudo apt update
sudo apt install -y curl
```

이후 아래 명령어로 Claude Code CLI를 설치합니다.

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

설치 후 아래 명령어로 Claude Code가 정상적으로 설치되었는지 확인합니다.

```bash
claude --version
```

<br>

## 2 ) Claude Code 실행 및 로그인
D-Racer 작업 디렉터리로 이동한 뒤 Claude Code를 실행합니다.

```bash
cd ~/D-Racer-Kit
claude
```

처음 실행하는 경우 로그인 또는 인증 절차가 진행됩니다.
화면에 표시되는 안내에 따라 Anthropic 계정 로그인을 진행합니다.

<br>

## 3 ) Claude Code 모델 및 추론 능력 설정
Claude Code는 세션 안에서 사용할 모델과 추론 노력도(effort)를 설정할 수 있습니다.
Claude Code를 실행한 뒤 입력창에서 slash command를 입력해 설정을 변경합니다.

### 3.1 /model
사용할 Claude 모델을 변경할 때 사용합니다.

```bash
/model
```

복잡한 설계, 디버깅, 코드 리뷰 작업은 `opus` 모델을 사용할 수 있습니다.

<br>

### 3.2 /effort
Claude Code가 문제를 얼마나 깊게 분석할지 설정할 때 사용합니다.

```bash
/effort
```

복잡한 버그 분석, 구조 변경, 큰 리팩토링처럼 더 많은 추론이 필요한 작업에서 높은 effort를 선택합니다.
간단한 수정이나 빠른 확인 작업에서는 낮은 effort를 선택할 수 있습니다.

<br>

### 3.3 /fast
빠른 응답이 필요한 경우 사용합니다.

```bash
/fast
```

`/fast`는 속도를 우선하는 작업에 적합합니다.
간단한 코드 수정, 명령어 확인, 문서 정리처럼 깊은 추론보다 빠른 응답이 중요한 경우 사용할 수 있습니다.

<br>

## 4 ) 이전 세션 다시 불러오기
Claude Code는 이전 대화 세션을 다시 불러와 이어서 작업할 수 있습니다.
가장 최근 세션을 이어서 실행하려면 아래 명령어를 사용합니다.

```bash
claude --continue
```

여러 세션 중 하나를 선택해서 다시 불러오려면 `--resume` 옵션을 사용합니다.

```bash
claude --resume
```

세션 안에서도 slash command로 설정 가능합니다.

```bash
/resume
```

<br>

## 5 ) 권한 확인 생략 모드 실행
Claude Code는 파일 수정이나 명령어 실행 전에 권한 확인을 요청할 수 있습니다.
실습용 샌드박스 환경처럼 작업 범위가 명확하고 위험이 낮은 경우 아래 옵션으로 권한 확인을 생략할 수 있습니다.

```bash
claude --dangerously-skip-permissions
```

이 옵션은 모든 권한 확인을 우회하므로, 인터넷 접근이 가능하거나 중요한 파일이 있는 환경에서는 사용하지 않는 것을 권장합니다.

권한 확인 생략 모드로 실행한 뒤, 필요한 경우 세션 안에서 `/model`, `/effort`, `/fast`를 사용해 모델과 추론 방식을 설정합니다.

권한 확인 생략 모드에서도 작업 전 현재 디렉터리가 올바른지 확인해야 합니다.

```bash
pwd
```

<br>

## 6 ) D-Racer 작업 예시
D-Racer 저장소에서 Claude Code를 실행하는 기본 예시는 아래와 같습니다.

```bash
cd ~/D-Racer-Kit
claude
```

Claude Code가 실행되면 필요에 따라 `/model`, `/effort`, `/fast`를 입력해 모델과 추론 방식을 설정합니다.

이전 작업을 이어서 진행하는 경우:

```bash
cd ~/D-Racer-Kit
claude --continue
```

샌드박스 실습 환경에서 권한 확인 없이 진행하는 경우:

```bash
cd ~/D-Racer-Kit
claude --dangerously-skip-permissions
```
