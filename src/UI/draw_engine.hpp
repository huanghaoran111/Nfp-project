#include<string>
#define LOG_LENGTH 512

namespace NFP{
    // DrawEngine是一个控制绘制模式的类
    struct DrawEngine{

        enum class DrawMode { 
            DRAWIMMEDIATELY,
            DRAWSTAGE
        };

        bool used = false;

        virtual std::string get_id() = 0;

        virtual void draw() = 0;

        DrawMode drawMode = DrawMode::DRAWIMMEDIATELY;

        char drawInfo[LOG_LENGTH] = {'\0'};

        struct Node
        {
            DrawEngine* instance = nullptr;

            ~Node(){
                free_node();
            }

            void free_node(){
                delete instance;
                instance = nullptr;
            }
            void set_used(bool used)
            {
                if (used) {
                    if (instance == nullptr) {
                        instance = create_instance();
                    }
                    instance->used = true;
                }
                else if (instance) {
                    instance->used = false;
                }
            }
            virtual DrawEngine *create_instance() = 0;
        };
    };

}