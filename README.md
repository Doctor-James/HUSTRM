# 2021.10.5 update
## 线程间传参分析
- 原来代码里进行线程间传参时，会将一个线程的指针传进另外的线程内以便进行数据交互，但是，这种方法会导致：
  - 扩展比较困难，添加或删除线程都会伴随着大量的修改
  - 耦合性过高，即代码之间不必要的关联很多，导致编译时间变长
- 那么是否存在一个不用改变传参，又能很好地解决信息交互的问题的方法呢？
  - 一种比较容易想到的思路是分出一个SharedData类作为共享数据的处理类
  - 由于这个类应该是所有线程的共有资源，并且只能有一份，可以考虑在父类使用静态指针来存储(静态成员在多个类的继承过程中只会存在一份)
  - 使用静态指针会伴随着释放内存的难题，这个可以由智能指针解决
    ```cpp
    class thread
    {
        static std::shared_ptr<SharedData> share_;
    };
    ```
    ```cpp
        //初始化
        std::shared_ptr<ShareData> thread::share_= std::shared_ptr<SharedData>(new SharedData())
    ```
## 关于滥用继承
- 我们知道，继承关系是c++一个很重要的特性，但是并不是类之间的关系都要用到继承，还有可能需要用到组合/聚合办法
- 那怎么区分继承和组合/聚合呢？举一个比较简单的例子
```cpp
    class son : public father
    {
        //此为继承方式
    };
```
```cpp
    class son
    {
        father *father_; //此为聚合/组合方式
    };
```
- 什么时候使用继承？下面是我从网上找到的一些原话，包含一些设计模式的内容
  1. 谨慎使用继承关系来进行扩展，优先使用聚合/组合方式
  2. 要使用继承，必须满足里氏代换原则，即父类出现的地方子类一定能够出现
  3. 继承对象的结构必须足够稳定，且继承的层次较浅。(足够稳定是指代码应该不怎么变化，比如虚基类)
  4. 当你需要使用继承关系时，肯定是想用其多态的特性
- 比如原代码里的LightBarThread是继承lightBar的，但实际上，组合的方式能够更加清晰地表示两者之间的关系。而且lightBar本身不是比较稳定的类，故不适合使用继承
## 关于依赖问题
- 可以简单地理解，如果包含了一个头文件，说明对该头文件所在的模块产生了依赖
- 常见的依赖问题
  - 依赖倒置
    - 比如原代码中，pnp模块依赖serialPortRead模块属于依赖倒置，原因是serialPortRead是由主函数部分直接依赖，属于比pnp模块层次更高的模块，依赖应该由高层次依赖低层次，而原代码里是高层次依赖低层次。如果pnp模块需要使用串口读取数据，那么可以由armorThread作为中间商作为传递
- 关于依赖我们要做到的目标是：
  - 层次分明
  - 尽量保持较低层次代码的稳定性，即尽可能使低层次的模块在未来改动最少
  - 一个板块，不应该依赖其它模块的内容，除非该板块需要依赖的部分稳定性很高，或者属于高层次的板块，比如任意模块可以依赖tool模块
  - 保持模块功能的专一化，必要时可以增加类。尽量不要在原功能的类内增加新的功能，这会对维护产生很大的麻烦。比如可将原来armor类分为两个部分，一个类用来计算装甲的特征，另一个类用来计算装甲特征的得分，结构会更加清晰
## 关于cmake编写细节
- 模板
```cmake
    find_package(OpenCV REQUIRED)
    include_directories(inc)
    aux_source_directory(src ARMOR_CPP) #这个用来设置整个文件的cpp集合，提高可扩展性
    add_library(
        armor
        ${ARMOR_CPP}
    target_link_libraries(
        armor
        tools
        ${OpenCV_LIBS})
                )
```
- 编写cmake需要注意的事项
    1. 尽可能使用相对路径
    2. 一个模块内的cmake链接这个模块需要的库，不要全部放在主模块的cmake来链接，这会导致库缺失时不容易找到位置
    3. 理论上主模块的cmake只需要链接其余模块产生的静态库，而不需要链接额外的库
        ```cmake
            target_link_libraries(
        armorDetector
        tools
        camera
        armor
        solvePNP
        thread
        )
        ```
# 2021.10.9 update
## 利用无锁循环队列来加快处理数据
- 舍弃了条件变量+锁的形式，因为这样会额外多两次复制操作，大大降低了速度
- 方案一：使用官方的无锁队列来进行编程
    ```cpp
    #include<boost/lockfree/spsc_queue.hpp>
    //图片队列创建
    boost::lockfree::spsc_queue<ly::Mat>pic_queue;
    //入队
    pic_queue.push(input_frame);
    //出队
    pic_queue.pop(output_frame);
    ```
  - 这个会有一个问题，就是可能产出图片和处理图片不同步，也就是如果处理图片比较慢的话图片会堆积导致实时性不够
  - 这个可以通过调整延时来达到弱同步，一定情况下可以缓解这个问题
- 方案二
  - 自己写的循环数组来对数据进行处理,其中利用到了原子操作，也包含动态内存模型的设吉计
  - 动态内存模型
    ```cpp
    std::memory_order::memory_order_relaxed：不强制代码的执行顺序
    std::memory_order::memory_order_acquire:强制后面的代码在这句代码之后执行，避免被编译器优化而改变执行顺序
    std::memory_order::memory_order_release：强制前面的代码在这句代码之前执行，避免编译器优化改变执行顺序
    ```