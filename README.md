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

