**사용법**
  -
PCL 화면에서 키 이벤트를 사용하여 각 시퀀스를 실행한다. 

1: Detectron2에서 학습한 것을 서버로 보내어 인식된 결과를 받아온다. 
<img width="1227" height="343" alt="image" src="https://github.com/user-attachments/assets/2b58eba2-f1d1-4cb2-bbc7-dc24ef1dd4b5" />

2: PCD데이터에서 다운 셈플링과 측면을 제거한 상태에서 RANSAC을 이용하여 바닥면을 제거한다.  
3: 남은 데이터를 저장하여 PCA를 실행하여 상대 좌표계를 구한다.
<img width="1231" height="390" alt="image" src="https://github.com/user-attachments/assets/98bc5ac5-d8ea-4605-a293-611c0d8b5239" />

4: 인식된 결과를 토대로 cobot이 물체를 파지하고 최종 목적지 까지 이동한다. 
<img width="1672" height="781" alt="image" src="https://github.com/user-attachments/assets/d6054e85-e39e-4249-8def-7f4595b77c3a" />

5: 위 시퀀스를 자동으로 실행  
0: 로봇의 초기위치로 이동 

각 시퀀스 순서
------

**1. Detectron2 물체인식**
<img width="960" height="1050" alt="RGB" src="https://github.com/user-attachments/assets/5b415fab-695a-48c4-a52e-015a3bb54801" />




**2. RANSAC 바닥 제거**
<img width="960" height="1050" alt="RANSAC" src="https://github.com/user-attachments/assets/d9bdb496-1b40-4246-bec6-93f551f0dc05" />



**3. PCA 방향 추정**
<img width="960" height="1050" alt="PCA" src="https://github.com/user-attachments/assets/d3b6ae8f-8e8b-4e18-9aa0-532561c1eaa8" />



**4. 로봇 이동**
<video src="https://github.com/user-attachments/assets/3b545f6e-c193-4177-a396-5ef372942714" controls></video>
