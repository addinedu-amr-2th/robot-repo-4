## 프로젝트 소개
![image](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/6ddb8631-cdef-4fba-96a8-c331b6a238a2)  
거대 언어 모델 **ChatGPT**를 이용한 로봇 행동 제어 프로젝트  
## 시연 영상
https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/4dd73bd7-c1d2-4d25-b1f7-6d03d8f8011c

## 목적
- AGI로도 생각되는 **거대 언어 모델(LLM)인 ChatGPT**를 사용하여 사용자의 요청을 해결하는 **로봇의 적절한 논리적인 행동리스트를 스스로 판단, 생성**시켜보자.
- 대화와 감정을 만들어 낼 수 있는 ChatGPT의 특성을 강조하여 인간과 로봇의 교감에 초점을 맞춘다.
- 기존에 없던 **ChatGPT를 이용해 로봇을 제어하는 새로운 프레임 워크** 개발
## 프로젝트 설명  
![image](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/50715f88-4995-45c1-b898-55eb4362f75f)
- 전체 시스템 구성도 : 사용자의 음성을 언어모델인 ChatGPT에 입력시키기 위해 google STT API를 사용, finetuning된 GPT 모델에 사용자의 요청에 해당하는 프롬프트를 넣고, 그 출력으로 나온 일련의 동작 시퀀스인 스케쥴을 스케쥴러에서 하나씩 parsing하여 여러 하드웨어들을 제어할 수 있도록 한다.
---
![image](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/5e9705f0-48b4-464d-b9ba-a34386d5ad65)
- finetuning : OpenAI에서 제공하는 서비스를 통해 입력 `prompt`와 모범답안 `completion`의 한 쌍으로 GPT 모델을 finetuning시켜 학습데이터의 말투, 문법, 스타일을 사용하는 나만의 GPT 모델을 만들 수 있다.
- 우리는 일련의 행동을 언어로 규정하기 위해서 기기, 행동, 파라미터를 콜론(:) 마크로, 동작과 동작 단위를 별(*) 마크로 정의하여 이를 기준으로 출력 스케쥴을 parsing할 수 있었다.
---
![image](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/25b1b54c-73ec-4eae-bad2-941725adcc03)
- 우리는 최적의 성능을 얻기 위해 위와 같은 기법들을 사용했다.
- 왼쪽(ChatGPT)과 오른쪽(GPT3)을 따로 놓은 이유는 OpenAI에서 제공하는 엔드포인트가 다르고, 사용하는 데이터 형태도 다르며, 모델의 성향조차 다르기 때문이다.
- ChatGPT의 경우 `system`을 통해 역할을 부여하면 이 역할을 지키려하는 역할극의 성향이 강했다.
  이 모델은 논리적인 행동 시퀀스에 해당하는 아웃풋을 냈지만, 지정한 문법에서 자꾸 벗어나는 문제가 존재했다.
- GPT 3의 경우, Few Shot Learning으로 몇 개의 예시를 들어주면, 해당 예시의 말투, 문법, 스타일을 따라하려는 경향이 강했다.
  이 모델은 비교적 지정한 문법에서 벗어나지 않았으나, 논리성은 떨어지는 모습을 보였다.
- 우리는 안정적인 시연을 위해 안정성을 중요시 했고, finetuning endpoint가 열려있는 GPT 3를 사용하게 됐다.
---
![image](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/1b09d285-7a48-4bc1-a114-fd63e38ba4dc)
- 이렇게 얻어진 행동 시퀀스는 Schedule Manager에 의해 ChatGPT 모델이 판단한 우선도에 따라 기존 스케쥴의 전방/후방에 배치되게 되고, Schedule Manager는 이를 하나씩 꺼내어 하드웨어에 명령을 내리고, 이 명령이 수행된 후 성공 여부를 나타내는 신호를 받아 이를 통해 하나의 동작의 완료를 인식하고 다음 동작을 순차적으로 실행하여 일련의 동작이 수행될 수 있게 하였다.
- 이론상 모든 복잡한 동작은 간단한 동작의 연속으로 구성되어 있으므로, 이는 ChatGPT가 사용자의 요구를 만족시키는 복잡한 동작들을 스스로 생성해냄을 의미한다.
---
![image](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/c1d2967f-4e4f-4783-9bb9-acd19d5cab1d)
- 위는 구체적인 동작의 예시이다.
- 사용자가 목이 마르다는 요청을 넣게 되면, ChatGPT는 이를 해소하기 위해 우선도를 높음으로 설정한 뒤, 
  병을 찾기, 병을 바라봄, 앞으로 감, 물병을 잡음, 사람을 찾음, 사람을 바라봄, 앞으로 감, 병을 내려놓음, 행복을 표현함의 일련의 과정을 출력으로 내놓는다.
- Schedule Manager는 이를 하나씩 다른 기기들에게 명령하게되고, 이로서 사용자의 요청에 대응하는 복잡한 동작이 이루어지게 된다.
---

그 결과 우리는 다음과 같이 사람과 친밀하게 상호작용하는 로봇을 만들 수 있었다.
![Animation](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/bf2a6e62-4831-43ac-bad3-9dc18b3c0750)
![Animation](https://github.com/addinedu-amr-2th/robot-repo-4/assets/69943723/c0a09812-fd23-4f6f-84f5-5903df42e91b)


---
## 의의


## 코드 설명
#### schedule_maker : 
#### stt_pkg : 
#### yolo_pkg : 
#### om_pkg : 
## 발표 자료
- [발표 PPT](https://docs.google.com/presentation/d/1Db-Mb1rRizueh5NoOPT9ax4vFm7Z98R1q-yJYuG1zGs/edit?usp=sharing)
## 팀원 소개
팀 GPT와 함께 춤을~
- [송승훈](https://github.com/addinedu-amr-2th/robot-repo-4/tree/ssh)
- [윤태웅](https://github.com/addinedu-amr-2th/robot-repo-4/tree/ytw)

