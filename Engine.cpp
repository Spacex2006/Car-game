#include "Engine.h"
#include <sstream> 
#include <iomanip> 
#include <algorithm> // [FIX]: Added for std::min

constexpr float PI = 3.14159f;

Engine::Engine() : window(sf::VideoMode(800, 600), "Multi-File Custom Engine") {
    window.setFramerateLimit(60);
    
    // Create a 1980kg test car at the center of the screen
    playerVehicle = new RigidBody(sf::Vector2f(400.f, 300.f), sf::Vector2f(20.f, 10.f), 1980.f);
    physics.registerBody(playerVehicle);

    if (!font.loadFromFile("font.ttf")) {
        // Fallback or warning if font is missing
    }
}

Engine::~Engine() {
    delete playerVehicle;
}

void Engine::run() {
    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        handleInput(dt);
        physics.update(dt);
        
        logger.recordFrame(dt, playerVehicle->motor, playerVehicle->rear, playerVehicle->velocity,playerVehicle->acceleration,playerVehicle->direction);
        render();
    }
    logger.exportToCSV("powertrain_telemetry_dump.csv");
}

void Engine::handleInput(float dt) {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
    }

    float chargeRate = 0.25f; // [FIX]: Sped this up so the pedal feels responsive 
    float decayRate = 2.0f;  

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
        playerVehicle->motor.pedalState += chargeRate * dt;
        if (playerVehicle->motor.pedalState > 1.0f) playerVehicle->motor.pedalState = 1.0f;
    }
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
        playerVehicle->motor.pedalState -= decayRate * dt;
        if (playerVehicle->motor.pedalState < -1.0f) playerVehicle->motor.pedalState = -1.0f;
    }
    else {
        if (playerVehicle->motor.pedalState > 0.05f) playerVehicle->motor.pedalState -= decayRate * dt;
        else if (playerVehicle->motor.pedalState < -0.05f) playerVehicle->motor.pedalState += decayRate * dt;
        else playerVehicle->motor.pedalState = 0.f;
    }
    
    // [GENERAL]: Calculate RPM based on wheel speed
    float rawRPM = std::abs(playerVehicle->rear.angularVelocity) * (30.0f / PI) * playerVehicle->motor.gearRatio;
    playerVehicle->motor.currRPM = std::min(rawRPM, playerVehicle->motor.maxRPM - 10.f);
    
    // Update motor state based on pedal input
    playerVehicle->motor.motorUpdate(playerVehicle->velocity,playerVehicle->direction);
    
    // =================================================================
    // [FIX]: TRACTION CONTROL SYSTEM (TCS)
    // Prevents the massive 4500 Nm axle torque from instantly breaking friction
    // =================================================================
    float maxGripForce = playerVehicle->rear.normalForce * playerVehicle->rear.staticFriction;
    float maxAllowedMotorTorque = (maxGripForce * playerVehicle->rear.radius) / playerVehicle->motor.gearRatio;

    if (std::abs(playerVehicle->motor.finalTorque) > maxAllowedMotorTorque) {
        float dir = (playerVehicle->motor.finalTorque >= 0) ? 1.0f : -1.0f;
        playerVehicle->motor.finalTorque = maxAllowedMotorTorque * dir;
    }
    
    // [GENERAL]: Mechanical multiplication through the reduction gear
    float adjustedTorque = playerVehicle->motor.finalTorque * playerVehicle->motor.gearRatio;
    
    // [FIX]: Pass the car's direction and mass into the wheel update!
    float f = playerVehicle->rear.wheelUpdate(adjustedTorque, playerVehicle->velocity, playerVehicle->direction, playerVehicle->mass, dt);
    
    // =================================================================
    // [FIX]: AERODYNAMIC DRAG (Direction-Aware)
    // =================================================================
    // Calculate longitudinal speed to know if we are moving forward or backward
    float longSpeed = (playerVehicle->velocity.x * playerVehicle->direction.x) + (playerVehicle->velocity.y * playerVehicle->direction.y);
    
    // Calculate absolute drag magnitude
    float aeroForceMag = 0.5f * 1.225f * 0.23f * 2.54f * (longSpeed * longSpeed);
    
    // Drag MUST always oppose the direction of travel
    float aeroDrag = (longSpeed > 0.f) ? -aeroForceMag : aeroForceMag;
    
    // Combine traction force and aero drag
    sf::Vector2f force = (f + aeroDrag) * playerVehicle->direction;
    
    playerVehicle->bodyUpdate(dt, force);
    playerVehicle->updateVisual();
}

void Engine::render() {
    window.clear(sf::Color(30, 30, 30));
    
    // Draw our vehicle using its internal visual representation
    window.draw(playerVehicle->visualShape);
    
    // =================================================================
    // [TELEMETRY UPDATE]: Recalculating physics states purely for the HUD
    // =================================================================
    
    // 1. Calculate Signed Speeds (to know if we are reversing)
    float longSpeed = (playerVehicle->velocity.x * playerVehicle->direction.x) + (playerVehicle->velocity.y * playerVehicle->direction.y);
    float accel = (playerVehicle->acceleration.x * playerVehicle->direction.x) + (playerVehicle->acceleration.y * playerVehicle->direction.y);

    // 2. Calculate Tire Slip Dynamics
    float wheelSurfaceSpeed = playerVehicle->rear.angularVelocity * playerVehicle->rear.radius;
    float slipV = wheelSurfaceSpeed - longSpeed;
    bool isSlipping = std::abs(slipV) >= 0.2f; // The threshold we defined in the wheel struct
    
    // 3. Calculate Aero Drag & TCS limits to show on the dashboard
    float aeroForceMag = 0.5f * 1.225f * 0.23f * 2.54f * (longSpeed * longSpeed);
    float aeroDrag = (longSpeed > 0.f) ? -aeroForceMag : aeroForceMag;
    
    float maxGripForce = playerVehicle->rear.normalForce * playerVehicle->rear.staticFriction;
    float maxAllowedMotorTorque = (maxGripForce * playerVehicle->rear.radius) / playerVehicle->motor.gearRatio;
    
    // 4. Check if TCS is actively choking the engine to save traction
    float rawTorque = playerVehicle->motor.throttle * playerVehicle->motor.currTorque;
    float pedalDir = (playerVehicle->motor.pedalState >= 0.f) ? 1.f : -1.f;
    bool tcsActive = std::abs(rawTorque) > maxAllowedMotorTorque && std::abs(playerVehicle->motor.pedalState) > 0.05f;

    // =================================================================
    // [TELEMETRY UPDATE]: Expanded string stream with formatted sections
    // =================================================================
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "===== LIVE POWERTRAIN TELEMETRY =====\n\n"
       
       << "[ DRIVER INPUT ]\n"
       << "Pedal State:    " << playerVehicle->motor.pedalState * 100.f << " %\n"
       << "Requested Pwr:  " << playerVehicle->motor.throttle * 100.f << " %\n\n"
       
       << "[ MOTOR & INVERTER ]\n"
       << "Motor RPM:      " << playerVehicle->motor.currRPM << " RPM\n"
       << "Raw Torque:     " << rawTorque * pedalDir << " Nm\n"
       << "TCS Limit:      " << maxAllowedMotorTorque << " Nm " << (tcsActive ? "<< ACTIVE" : "") << "\n"
       << "Delivered Trq:  " << playerVehicle->motor.finalTorque << " Nm\n\n"
       
       << "[ TIRE DYNAMICS (REAR) ]\n"
       << "Surface Speed:  " << std::abs(wheelSurfaceSpeed * 3.6f) << " km/h\n"
       << "Ground Speed:   " << std::abs(longSpeed * 3.6f) << " km/h\n"
       << "Slip Velocity:  " << slipV << " m/s\n"
       << "Tire State:     " << (isSlipping ? "SLIPPING (Kinetic)" : "GRIPPING (Static)") << "\n\n"
       
       << "[ CHASSIS KINEMATICS ]\n"
       << "Aero Drag:      " << aeroDrag << " N\n"
       << "Acceleration:   " << accel / 9.81f << " G\n"
       << "Velocity:       " << std::abs(longSpeed * 3.6f) << " km/h";

    sf::Text hudText;
    hudText.setFont(font);
    hudText.setString(ss.str());
    hudText.setCharacterSize(15); // [FIX]: Slightly smaller text to fit all the new data
    hudText.setFillColor(sf::Color::White);
    hudText.setPosition(10.f, 10.f); 

    // [FIX]: Expanded the background box to fit the new lines
    sf::RectangleShape hudBackground(sf::Vector2f(310.f, 440.f));
    hudBackground.setFillColor(sf::Color(0, 0, 0, 200)); // Slightly darker for readability
    hudBackground.setPosition(5.f, 5.f);
    
    window.draw(hudBackground);
    window.draw(hudText);
    
    window.display();
}