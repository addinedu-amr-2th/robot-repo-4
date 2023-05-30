# Project schedule

### 사전 준비
- 원래 [Deepspeech stt 모델](https://github.com/sooftware/kospeech)을 이용하여 학습해 사용하려 했으나, 컴퓨팅 파워 부족으로 [이미 학습되어 있는 딥러닝 모델](https://github.com/kakaobrain/pororo)을 사용하기로 결정. 성능은 양호해보인다. 
- ChatGPT 사전 리서치 : 비용문제 때문에 연습용으로는 curie 모델을 fine-tuning해서 사용하고, 이후 시연시 davich 모델을 fine-tuning해서 사용하게 될 것으로 생각됨. 이를 위한 fine-tune 데이터셋은 어느정도 제작해둠(시나리오 추가 요구)
- TTS 모델은 눈여겨봐둔 모델이 존재하나 다른 모델을 사용하게 될 것으로 생각됨 

### 2023.5.30
- [stt 활용](https://github.com/sooftware/kospeech#introduction) --> whisper 사용하여 녹음파일 텍스트화 성공!

- [audioROS](https://github.com/LCAV/audioROS) 패키지를 활용해서 오디오 퍼블리셔, 서브스크라이버 제작. 일정 시간 퍼블리셔가 구동하다 정지하는 현상 해결
  하지만, 음성이 뚝뚝 끊기는 문제 발생. ROS통신의 속도에 한계가 존재해서 딜레이를 청크단위로 끊어서 그런 것으로 생각됨.
  이 문제를 해결하기 위해선 2가지 솔루션이 존재할 것으로 예상됨
  - 1. 한번에 보내는 사이즈를 키워서 슬로우 모션으로 들리는 상태로 STT 모델에 건네주는 것 (이 경우 느린 음원을 STT 모델이 제대로 알아들을까 미지수)
  - 2. 서브스크라이버 측에서 짧은 시간동안 청크를 모아서 짧은 음원을 만들고 STT 모델에 넣어 1,2 글자를 뱉게 해서 이것을 하나의 문장으로 모으는 것
    (주로 [realtime STT](https://github.com/davabase/whisper_real_time/blob/master/transcribe_demo.py)가 이런 방식을 채택하는 것으로 보임.) 
