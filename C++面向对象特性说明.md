# 项目中C++面向对象特性体现

## 1. 封装（Encapsulation）

### 访问控制：public / private / protected

**示例1：PNCMapCreatorBase（基类封装）**
```cpp
class PNCMapCreatorBase {
public:
    virtual PNCMap create_pnc_map() = 0;  // 公共接口
    inline PNCMap pnc_map() const {return pnc_map_;}  // 公共访问器

protected:  // 保护成员，子类可访问，外部不可访问
    std::unique_ptr<ConfigReader> pnc_map_config_;
    PNCMap pnc_map_;
    MarkerArray pnc_map_markerarray_;
    // ... 其他保护成员
};
```

**示例2：PNCMapCreatorStraight（私有方法封装）**
```cpp
class PNCMapCreatorStraight : public PNCMapCreatorBase {
public:
    PNCMapCreatorStraight();
    PNCMap create_pnc_map() override;  // 公共接口

private:  // 私有方法，仅类内部使用
    void init_pnc_map();  // 初始化方法
    void draw_straight_x(...);  // 绘制方法
};
```

**体现**：
- 数据成员隐藏在`protected/private`中
- 通过`public`接口提供访问
- 实现细节被封装，外部无法直接访问

---

## 2. 继承（Inheritance）

### 基类和派生类关系

**示例1：地图创建器继承体系**
```cpp
// 基类
class PNCMapCreatorBase {
    virtual PNCMap create_pnc_map() = 0;
protected:
    PNCMap pnc_map_;  // 子类可访问
};

// 派生类1：直道地图
class PNCMapCreatorStraight : public PNCMapCreatorBase {
public:
    PNCMap create_pnc_map() override;  // 重写基类方法
};

// 派生类2：S弯地图
class PNCMapCreatorSTurn : public PNCMapCreatorBase {
public:
    PNCMap create_pnc_map() override;  // 重写基类方法
};
```

**示例2：车辆类继承体系**
```cpp
// 基类
class VehicleBase {
public:
    virtual void vehicle_cartesian_to_frenet(...) = 0;
protected:
    double length_, width_;  // 子类继承
};

// 派生类1：主车
class MainCar : public VehicleBase {
public:
    void vehicle_cartesian_to_frenet(...) override;
};

// 派生类2：障碍车
class ObsCar : public VehicleBase {
public:
    void vehicle_cartesian_to_frenet(...) override;
};
```

**体现**：
- 使用`public`继承，实现"是一个"关系
- 子类继承基类的成员变量和方法
- 代码复用，避免重复定义

---

## 3. 多态（Polymorphism）

### 虚函数和运行时多态

**示例1：多态调用地图创建器**
```cpp
// pnc_map_server.cpp 第45-58行
void PNCMapServer::response_pnc_map_callback(...) {
    switch (request->map_type) {
        case STRAIGHT:
            map_creator_ = std::make_shared<PNCMapCreatorStraight>();  // 创建直道
            break;
        case STERN:
            map_creator_ = std::make_shared<PNCMapCreatorSTurn>();  // 创建S弯
            break;
    }
    
    // 多态调用：通过基类指针调用，实际执行派生类方法
    const auto pnc_map = map_creator_->create_pnc_map();  // 运行时确定调用哪个版本
}
```

**示例2：车辆多态处理**
```cpp
// planning_process.cpp 第325行
for (const auto &obs : obses_) {
    obs->vehicle_cartesian_to_frenet(refer_line);  // 多态调用
    // 实际调用的是ObsCar::vehicle_cartesian_to_frenet()
}
```

**体现**：
- 基类定义虚函数接口
- 派生类使用`override`重写
- 通过基类指针/引用调用，运行时确定实际调用的方法

---

## 4. 抽象类（Abstract Class）

### 纯虚函数定义接口

**示例1：PNCMapCreatorBase（抽象基类）**
```cpp
class PNCMapCreatorBase {
public:
    virtual PNCMap create_pnc_map() = 0;  // 纯虚函数，抽象接口
    virtual ~PNCMapCreatorBase() {}  // 虚析构函数
};
```

**示例2：VehicleBase（抽象基类）**
```cpp
class VehicleBase {
public:
    virtual void vehicle_cartesian_to_frenet(...) = 0;  // 纯虚函数
    virtual void vehicle_cartesian_to_frenet_2path(...) = 0;  // 纯虚函数
    virtual ~VehicleBase() {}  // 虚析构函数
};
```

**体现**：
- 使用`= 0`定义纯虚函数
- 不能实例化抽象类
- 强制派生类实现接口
- 定义统一的接口规范

---

## 5. 虚析构函数（Virtual Destructor）

**示例：**
```cpp
class PNCMapCreatorBase {
public:
    virtual ~PNCMapCreatorBase() {}  // 虚析构函数
};

class VehicleBase {
public:
    virtual ~VehicleBase() {}  // 虚析构函数
};
```

**体现**：
- 通过基类指针删除派生类对象时，确保调用正确的析构函数
- 防止内存泄漏
- 体现良好的C++编程实践

---

## 6. override关键字（C++11）

**示例：**
```cpp
class PNCMapCreatorStraight : public PNCMapCreatorBase {
public:
    PNCMap create_pnc_map() override;  // 明确表示重写基类方法
};

class MainCar : public VehicleBase {
public:
    void vehicle_cartesian_to_frenet(...) override;  // 明确表示重写
};
```

**体现**：
- 明确标识重写基类方法
- 编译器检查，防止拼写错误
- 提高代码可读性和安全性

---

## 7. 智能指针管理资源

**示例：**
```cpp
// 使用智能指针管理对象
std::shared_ptr<PNCMapCreatorBase> map_creator_;
std::unique_ptr<ConfigReader> pnc_map_config_;
std::shared_ptr<VehicleBase> car_;
```

**体现**：
- 使用`std::shared_ptr`实现共享所有权
- 使用`std::unique_ptr`实现独占所有权
- 自动内存管理，避免内存泄漏
- 现代C++最佳实践

---

## 8. 命名空间（Namespace）

**示例：**
```cpp
namespace Planning {
    class PNCMapCreatorBase { ... };
    class VehicleBase { ... };
}
```

**体现**：
- 避免命名冲突
- 组织代码结构
- 模块化设计

---

## 9. 内联函数（Inline Functions）

**示例：**
```cpp
class VehicleBase {
public:
    inline double length() const { return length_; }  // 内联访问器
    inline double width() const { return width_; }
    inline double s() const { return s_; }
};
```

**体现**：
- 提高访问器函数性能
- 减少函数调用开销
- 保持封装性的同时优化性能

---

## 10. const成员函数

**示例：**
```cpp
class PNCMapCreatorBase {
public:
    inline PNCMap pnc_map() const {return pnc_map_;}  // const成员函数
    inline MarkerArray pnc_map_markerarray() const {return pnc_map_markerarray_;}
};

class VehicleBase {
public:
    inline double length() const { return length_; }  // const成员函数
    inline double speed() const { return speed_; }
};
```

**体现**：
- 保证成员函数不修改对象状态
- 可以在const对象上调用
- 提高代码安全性

---

## 实际应用场景总结

### 场景1：地图创建器多态
```cpp
// 服务器根据请求类型，动态创建不同的地图创建器
map_creator_ = std::make_shared<PNCMapCreatorStraight>();  // 或STurn
auto map = map_creator_->create_pnc_map();  // 多态调用
```

### 场景2：车辆统一处理
```cpp
// 统一处理主车和障碍车
std::vector<std::shared_ptr<VehicleBase>> vehicles;
vehicles.push_back(car_);  // MainCar
vehicles.push_back(obs_);  // ObsCar

for (auto &v : vehicles) {
    v->vehicle_cartesian_to_frenet(refer_line);  // 多态调用
}
```

### 场景3：规划器扩展
```cpp
// 全局路径规划器基类，支持扩展
class GlobalPlannerBase {
    virtual Path search_global_path(...) = 0;
};

// 可以轻松添加新的规划器
class GlobalPlannerAStar : public GlobalPlannerBase { ... };
```

---

## 面向对象设计优势

1. **可扩展性**：通过继承和多态，易于添加新功能
2. **可维护性**：封装隐藏实现细节，修改不影响外部
3. **代码复用**：基类定义通用功能，子类复用
4. **接口统一**：抽象类定义统一接口，便于替换实现
5. **类型安全**：编译时类型检查，减少运行时错误

---

## 面试要点总结

**一句话总结**：项目充分运用了C++面向对象的**封装、继承、多态**三大特性，通过抽象基类定义接口，派生类实现具体功能，使用多态实现运行时动态选择，体现了良好的面向对象设计思想。

**核心亮点**：
1. **抽象基类设计**：PNCMapCreatorBase、VehicleBase等定义统一接口
2. **多态应用**：服务器根据请求类型动态选择地图创建器
3. **封装保护**：使用protected/private保护内部实现
4. **现代C++特性**：override、智能指针、const成员函数

