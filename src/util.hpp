#include<string>
#include<functional>
#include<any>

struct EventActivator{
    static EventActivator& GetInstance() {
        static EventActivator instance;
        return instance;
    }
    EventActivator(const EventActivator&) = delete;
    EventActivator& operator=(const EventActivator&) = delete;
    void RegisterEvent(const std::string& eventName, std::function<void()> callback) {
        m_eventCallbacks[eventName] = callback;
    }
    template<typename... Args>
    void RegisterEvent(const std::string& eventName, std::function<void(Args...)> callback) {
        m_eventCallbacks[eventName] = CallbackWrapper<Args...>{callback};
    }
    bool ActivateEvent(const std::string& eventName) {
        auto it = m_eventCallbacks.find(eventName);
        if (it != m_eventCallbacks.end()) {
            if (auto* callback = std::any_cast<std::function<void()>>(&it->second)) {
                (*callback)();
                return true;
            }
        }
        return false;
    }
    template<typename... Args>
    bool ActivateEvent(const std::string& eventName, Args... args) {
        auto it = m_eventCallbacks.find(eventName);
        if (it != m_eventCallbacks.end()) {
            try {
                auto& wrapper = std::any_cast<CallbackWrapper<Args...>&>(it->second);
                wrapper.callback(args...);
                return true;
            } catch (const std::bad_any_cast&) {
                // 类型不匹配
                return false;
            }
        }
        return false;
    }
    bool HasEvent(const std::string& eventName) const {
        return m_eventCallbacks.find(eventName) != m_eventCallbacks.end();
    }

    void RemoveEvent(const std::string& eventName){
        this->m_eventCallbacks.erase(eventName);
    }
private:
    template<typename... Args>
    struct CallbackWrapper {
        std::function<void(Args...)> callback;
    };
    EventActivator() = default; // 私有构造函数
    ~EventActivator() = default;
    std::unordered_map<std::string, std::any> m_eventCallbacks;
};