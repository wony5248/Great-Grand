## Members 
#### 신은지
#### 윤익선
#### 이지훈
#### 이희재
#### 장범진


## Naming Rules
### 1. Commit Message
#### Commit Rule
```
<type>: <subject>       -- 헤더
<BLANK LINE>
<body>                  -- 본문
<BLANK LINE>
<footer>                -- 바닥글
```
* type: 어떤 의도로 커밋했는지 명시
* subject: 최대 50글자가 넘지 않도록 작성
    * 한글로 작성하는 경우
        1. 처음은 동사 원형으로 시작
        2. 마침표, 느낌표 등의 특수문자는 작성하지 않음
    * 영어로 작성하는 경우
        1. 첫 글자는 대문자로 작성
        2. "Fix", "Add", "Change" 등의 동사로 시작
* body: 긴 설명이 필요할 경우 작성 (**무엇을 왜**)
* footer: issue tracker ID 를 명시하고 싶은 경우 작성

#### Type
* **Feat** : 새로운 기능을 추가할 경우
* **Build** : 빌드 관련 파일을 수정할 경우
* **Fix** : 버그를 수정할 경우
* **Design** : CSS등 사용자 UI 디자인을 변경할 경우
* **Chore** : 그 외 자잘한 수정에 대한 커밋
* **Ci** : CI관련 설정 수정에 대한 커밋
* **Docs** : 문서 수정에 대한 커밋
* **Style** : 코드 스타일 혹은 포맷 등에 관한 커밋
* **Refactor** : 코드 리팩토링
* **Rename** : 파일 혹은 폴더명을 수정하거나 옮길 경우
* **Remove** : 파일을 삭제하는 작업만 수행할 경우
* **Test** : 테스트 코드 추가, 테스트 코드 리팩토링
