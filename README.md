# 코드 소개

# ROS 프로젝트 개발일지
### 사전 준비
- 원래 [Deepspeech stt 모델](https://github.com/sooftware/kospeech)을 이용하여 학습해 사용하려 했으나, 컴퓨팅 파워 부족으로 [이미 학습되어 있는 딥러닝 모델](https://github.com/openai/whisper)을 사용하기로 결정. 성능은 양호해보인다. 
- ChatGPT 사전 리서치 : 비용문제 때문에 연습용으로는 curie 모델을 fine-tuning해서 사용하고, 이후 시연시 davich 모델을 fine-tuning해서 사용하게 될 것으로 생각됨. 이를 위한 fine-tune 데이터셋은 어느정도 제작해둠(시나리오 추가 요구)
- TTS 모델은 눈여겨봐둔 모델이 존재하나 다른 모델을 사용하게 될 것으로 생각됨 

### 2023.5.30
- [audioROS](https://github.com/LCAV/audioROS) 패키지를 활용해서 오디오 퍼블리셔, 서브스크라이버 제작. 일정 시간 퍼블리셔가 구동하다 정지하는 현상 해결
  하지만, 음성이 뚝뚝 끊기는 문제 발생. ROS통신의 속도에 한계가 존재해서 딜레이를 청크단위로 끊어서 그런 것으로 생각됨.
  이 문제를 해결하기 위해선 2가지 솔루션이 존재할 것으로 예상됨
  - 1. 한번에 보내는 사이즈를 키워서 슬로우 모션으로 들리는 상태로 STT 모델에 건네주는 것 (이 경우 느린 음원을 STT 모델이 제대로 알아들을까 미지수)
  - 2. 서브스크라이버 측에서 짧은 시간동안 청크를 모아서 짧은 음원을 만들고 STT 모델에 넣어 1,2 글자를 뱉게 해서 이것을 하나의 문장으로 모으는 것
    (주로 [realtime STT](https://github.com/davabase/whisper_real_time/blob/master/transcribe_demo.py)가 이런 방식을 채택하는 것으로 보임.) 

### 2023.5.31
- 오디오 퍼블리셔 토픽 발행 주기를 짧게 하니, 오디오가 끊기는 문제 해결. 소리가 반복해서 들리나, 이는 스피커에서 나온 소리가 다시 마이크로 들어가 생기는 메아리 현상으로 보인다.
  실제 로봇에 적용하려면 TTS로 말하는 동안에는 음성 입력을 받지 않는 상태로 만들어야 할 것이다(까먹지 말자)
  현재 음성 전송을 잘 하기 위한 조건은 다음과 같다
   - 발행하는 쪽의 chunk size는 작을 수록 매끈하다.
   - 발행하는 주기를 짧게 해야 매끈하게 전송된다.

- 현재 3개의 패키지를 만들었다.
    - audio publisher : 음성을 퍼블리쉬 하는 패키지 (청크 단위로 보내는 stream 모듈이 존재)
      - file : tts 모델이 추가된 하위 모듈이 만들어질 것으로 예상 - 그 때 직접 만들어야 할듯
    - audio subscriber : 음성을 받아서 재생하는 패키지 (현재 음성을 받아들이는 audio subsciber 모듈만 존재, 여기에 stt까지 적용시킨 하위 모듈을 추가할 예정)
    - audio inference : msg등 필요한 도구들이 모여있는 패키지 (위의 두 패키지를 실행하기 위해서 필요)

- STT 모델을 audio subscriber에 넣어 stt_subscriber 모듈을 만들어 봤는데, 주피터로 음성을 녹음해서 해독시켰을 때는 정상 해독했으나
  callback함수로 chunk를 모으다 보내는 방식으로 해봤더니 이상하게 번역하는 문제 발생
  subscriber쪽 문제일텐데 어떻게 수정해야 제대로 알아들을지 감이 안온다..
  - Hello라고 말하는데 I'm Sorry라고 해석한다.
 
### 2023.6.1
- 잡음의 해결법은 soundfile이라는 모듈.
  메세지로 음성을 보내다보니 보내지는 파일은 numpy로 보내지는데 이를 제대로 파일로 저장하는 방법은 이 모듈밖에 없었던 듯 하다.
- STT 모델이 잡음도 멋대로 Transcript해버리는게 골치아프다. 잡음은 좀 모델한테 안보내면 좋겠는데.. numpy 청크 수치를 0로 바꾸면 되려나.. 모르겠다.
- Whisper가 단어단위로 학습된 모듈이라 단어 vocab에 없으면 번역이 제대로 안돼서 고유명사나 가.나.다 같은건 번역이 제대로 안된다.
  언어모델에 전해줄 로봇의 이름을 그냥 로봇이라고 해야할듯..
- 매니퓰레이터, 뎁스 카메라가 존재하는 터틀봇4를 제공받을 수 있을 것 같다. 당분간 터틀봇4를 사용할 수 있게끔 하는게 목표가 될지도

- 일단 1차적인 STT 모듈은 만들어진 것 같다. 이 모듈에 추가해야 하는 기능은
    - 1. 주변 환경음 소리를 없애버리기 (numpy 행렬 개조하면 될지도)
    - 2. 자기 이름이 들릴 때까지는 소리를 듣는 시간을 짧게(시동코드), 이름이 들리고 나면 듣는 시간을 길게 설정하는 부분(실질적 GPT에 넣을 인풋)
    - 3. STT 결과물을 그냥 툭 뱉고 끝이 아니라 스택에 저장해서 길게 만들어 사용하는 부분 (I should buy, an apple from, market => I should buy an apple from market)
    - 4. 그렇게 모인 스택을 GPT API에 넣고 결과물 텍스트를 메세지로 발행하는 부분.. 근데 기본 Subscriber 모듈을 개조해서 만든거라, 여기서 발행까지 되게 하는게 은근 힘들지도.. 

### 2023.6.2
- String msg를 받아 여러 액션을 수행하는 action_maker 패키지를 제작.
  - 인위적으로 sleep을 주지 않으면 바로 다음 스케쥴로 넘어가 이전 스케쥴이 시행되지 못하는 문제가 존재
  - 이 것을 해결하기 위해선 각 스케쥴을 액션으로 처리해서 지속적인 동작과 완료 여부를 알려줘야만 한다.
  - ROS의 액션에 대해서 공부하고 파일을 개조해야..
- STT에서 ChatGPT 모델이 없을 때(비정상상태) 어느정도 기본 동작은 가능하게 초안 개조할 예정

### 2023.6.7
- 액션으로 개조하려 했으나 기초적인 수준의 동작은 완료 시점이 정의되지 않아 그냥 딜레이로 처리해야하는 것을 깨달음. 복잡한 동작의 경우에만 액션으로 처리해야할 것 같다.
- led, 부저 등 low 레벨의 동작을 구현하려 했으나 실패, ROS 아두이노 제어 방법 구조를 파악하기 위해 분석

### 2023.6.8
- action maker : 기본적인 동작 정의, 메세지 받아서 순차 처리하게 정리
- stt_subscriber : gpt 모델이 정상작동하지 않을 경우 대비 코드 동작 확인 완료.
- whisper 모델이 transcript 뿐 아니라 번역작업까지 한다는 사실을 발견, 언어 제한적이지 않은 서비스가 가능할 것 같다.

### 2023.6.9
- 우선순위 높은 명령이 들어올 경우 처리할 수단이 필요. 
  스케쥴을 지금처럼 명령이 들어오면 덮어씌우는 방식이 아니라 만약 우선순위 높은 명령이 들어오면 앞으로 추가, 아니면 뒤로 추가하는 방식으로 스케쥴 메이커 수정할 필요성이 생겼다.
- ChatGPT가 설계할 로직의 '코드블록'을 위해서 시나리오를 생각하자.
- Turtlebot4 제어 테스트
- 시스템 설계를 위한 UML?로는 '시스템 구성도(H/W(io), S/W)'와 예상기능리스트(가능할 것으로 예상되는 동작)

### 2023.6.13
- turtlebot4 구동 설정 성공. 로봇팔이 가능해질 것 같다.
- 이제 chatgpt API 맥락, 피드백 사용로직 및 fine tuning 준비에 들어갈듯하다
- action maker 명령 우선순위 로직 및 여러 파라미터 받을 수 있게 디폴트 값 추가해서 개조가 필요
