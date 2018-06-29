# xiaoqiang_audio_controller
xiaoqiang audio controller 通过语音控制机器人运动

### 安装

```bash
git clone https://github.com/BluewhaleRobot/xiaoqiang_audio_controller
```

### 使用

```bash
roslaunch xiaoqiang_audio_controller audio_controller.launch
```

这个launch文件依赖于 [xiaoqiang_tts](https://github.com/BluewhaleRobot/xiaoqiang_tts), [xiaoqiang_nlp](https://github.com/BluewhaleRobot/xiaoqiang_nlp) 
和[xiaoqiang_audio](https://github.com/BluewhaleRobot/xiaoqiang_audio)

### 消息类型

|输入话题|消息类型|说明|
|:--|:--|:--|
|～listen|std_msgs/String|需要识别的文本指令|

|输出话题|消息类型|说明|
|:--|:--|:--|
|~talk|std_msgs/String|输出到语音合成节点，发出语音|
|~chat|std_msgs/String|输出到自然语言处理节点，和用户聊天|
|~cmd|galileo_msgs/GalileoNativeCmd|输出之伽利略导航控制节点，控制小强导航|

### 语音指令

|指令|说明|
|:--|:--|
|开始导航|开启视觉导航功能|
|到X点|到X号目标点|
|取消，暂停，继续|取消，暂停，继续当前任务|
|关闭导航|关闭视觉导航功能|
|关机|关机|

### 使用教程

请参考[蓝鲸技术论坛](https://community.bwbot.org/topic/493/%E4%BD%BF%E7%94%A8%E8%AF%AD%E9%9F%B3%E6%8E%A7%E5%88%B6%E5%AF%BC%E8%88%AA)
