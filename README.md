# 일회용 마스크 쓰래기 수집 장치
* * *
> * 코로나 시대 전세계 마스크 쓰레기 매월 1290억개 발생
>
> * 이로 인한 환경 문제와 재활용 문제를 해결 방안으로 마스크 수집 장치를 제안 
>
> * 마스크 수집 참여도를 높이기 위하여 게이미피케이션을 적용
>
> * 수집된 마스크 화학 처리를 통해서 플라스틱으로 재활용 (프랑스 업체) 
>
> * 국내 청년이  마스크 150개를 녹여 의자로 만든 사례가 존재하는 등 재활용 연구가 활발히 이루어짐
 
# 시연 동영상 (그림 클릭)

[![Watch the video](https://t1.daumcdn.net/cfile/tistory/994CA84A5FAE29F33B?download)](https://youtu.be/6W6Lan_oPBM)


# Hell Maker (김도혁/김경남/유수엽/강진호/임재준/신태욱)
* * *
> * 김도혁 : 기획 / Web API 연동 페이지 제작
>
> * 김경남 : STM32 제어 보드
>
> * 유수엽 : STM32 제어 보드
>
> * 강진호 : STM32 제어 보드
>
> * 임재준 : 디자인 담당 / 영상 편집
>
> * 신태욱 : 비전 담당


## 전체 구성도
* * *

![슬라이드10](https://user-images.githubusercontent.com/46912845/103421172-53760b00-4bde-11eb-9964-9a4cfe4f85fe.JPG)

![슬라이드11](https://user-images.githubusercontent.com/46912845/103421173-53760b00-4bde-11eb-9343-ffe579411246.JPG)

![슬라이드12](https://user-images.githubusercontent.com/46912845/103421174-540ea180-4bde-11eb-93e0-33dffa5a5edc.JPG)


## 영상 인식 장치
* * *
<img src="https://user-images.githubusercontent.com/46912845/103461526-569a0400-4d62-11eb-9d0c-179a74019b58.jpg" width="50%"  title="영상 인식 장치" alt="영상 인식 장치"></img>

### 제작 도구 
 > * 유니티

### 주요 기능
 > * 카메라를 통하여 마스크 인식 후 (화면 좌측 하단)
 > * Bluetooth로 STM32에 메시지 전송
 > * 추가 포인트를 얻기 위한 게임 (게이미 피케이션)

### 특이 사항
 > * 마스크 관련한 마땅한 학습 데이타가 없어 직접 제작 하였으며, 인식률 향상을 위해서는 추가 학습이 필요함
 
 ## 마스크 수집 장치
* * *


### 제작 도구 
 > * STM32

### 주요 기능
 
 > * 영상 입력 장치와 Bluetooth통신 통한 명령 처리 (통신 규약 정의)  
 > * 수집 장치 On/Off
 > * UV 장치 On/Off
 > * 1분 단위 기기 상태 수집 (사용량,온도,습도등) 서버에 전송
 

### Dash Board  (Web Site)
 > * STM32

 
             
             
             
