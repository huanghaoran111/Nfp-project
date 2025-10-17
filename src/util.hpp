#pragma once
#include<string>
#include<functional>
#include<any>

#include<MathCore.h>
#include<Shape.hpp>
#include <chrono>

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


using tranglationPoints = std::tuple<NFP::Vec2, NFP::Vec2, NFP::Vec2>;

std::vector<tranglationPoints> delaunay_triangulation(std::vector<std::shared_ptr<NFP::Point>> points);

std::vector<std::vector<std::shared_ptr<NFP::Point>>> getDataFromJson(const std::string& jsonPath);

class CodeTimer {
public:
    explicit CodeTimer(const std::string& name = "", int sampleCount = 50) 
        : name_(name), sampleCount_(sampleCount) {}
    
    ~CodeTimer() {
        if (count_ > 0) {
            printStats();
        }
    }
    
    template<typename Func>
    void measure(Func&& func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        count_++;
        totalDuration_ += duration;
        
        if (count_ >= sampleCount_) {
            printStats();
            reset();
        }
    }
    
    void reset() {
        count_ = 0;
        totalDuration_ = std::chrono::microseconds(0);
    }
    
private:
    void printStats() {
        std::cout << name_ << " - Average time per run: " 
                  << totalDuration_.count() / count_ << " us (" 
                  << count_ << " samples)\n";
    }
    
    std::string name_;
    int sampleCount_;
    int count_ = 0;
    std::chrono::microseconds totalDuration_{0};
};

#define TimingExp(express) if(EventActivator::GetInstance().HasEvent("TimingAlgo")) {CodeTimer timer("Algo Time"); timer.measure([&]() { express; }); } else {do{express;}while(0); }

std::vector<NFP::Point> MoveNFPFunc(std::vector<std::vector<std::shared_ptr<NFP::Point>>>&);