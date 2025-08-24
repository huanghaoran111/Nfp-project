#include <Shape.hpp>


ShapeID::ShapeID()
    :width(std::to_string(std::numeric_limits<unsigned int>::max()).length()) {}

ShapeID* ShapeID::getInstance(){
    static ShapeID* instance = nullptr;
    std::lock_guard<std::mutex> lock(mutex);
    if(instance == nullptr){
        instance = new ShapeID();
    }
    return instance; 
}

std::string ShapeID::generate(const std::string& prefix) {
    std::lock_guard<std::mutex> lock(ShapeID::mutex);
    std::ostringstream oss;
    auto& count = counterMap[prefix];
    oss << prefix 
        << "-" 
        << std::setw(this->width) 
        << std::setfill('0') 
        << count++;
    return oss.str();
}

void ShapeID::reset(unsigned int startValue) {
    std::lock_guard<std::mutex> lock(ShapeID::mutex);
    counterMap.clear();
}

Shape::Shape(const std::string& shape_name) : SHAPE_NAME{shape_name} {
    if(Shape::idGenerator == nullptr) {
        Shape::idGenerator = ShapeID::getInstance();
    }
    id = Shape::idGenerator->generate(SHAPE_NAME);
}

std::string Shape::getType() const {
    std::string pattern = "^([^0-9]*)(\\d{" + std::to_string(std::numeric_limits<unsigned int>::max()) + "})$";
    std::regex sepRegex(pattern);
    std::smatch matches;
    if (std::regex_match(id, matches, sepRegex)){
        return matches[1];
    }
    return "Unknown";
}

std::string Shape::getId() const { return id; }

Line::Line(): Shape("Line") {
    
}

void Line::draw() const{
    std::cout << "Drawing a line" << std::endl;
}