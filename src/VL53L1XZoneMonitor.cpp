// VL53L1X Monitor Library - Implementation File
// GPL License
// This file contains the implementation of the VL53L1XZoneMonitor and ZoneObserver classes.

#include "VL53L1XZoneMonitor.h"

ZoneObserver::ZoneObserver(uint16_t min, uint16_t max, void (*onEnter)(uint16_t), void (*onExit)())
    : min_distance(min), max_distance(max), object_present(false), on_enter(onEnter), on_exit(onExit),
      in_zone_count(0), out_zone_count(0) {}

void ZoneObserver::evaluate(uint16_t distance, size_t certainty)
{
    bool in_zone = (distance >= min_distance && distance <= max_distance);
    if (in_zone)
    {
        in_zone_count++;
        out_zone_count = 0;
        if (in_zone_count >= certainty && !object_present)
        {
            object_present = true;
            if (on_enter)
                on_enter(distance);
        }
    }
    else
    {
        out_zone_count++;
        in_zone_count = 0;
        if (out_zone_count >= certainty && object_present)
        {
            object_present = false;
            if (on_exit)
                on_exit();
        }
    }
}

bool ZoneObserver::isObjectPresent() const
{
    return object_present;
}

VL53L1XZoneMonitor::VL53L1XZoneMonitor(TwoWire *wire, uint32_t interval_ms, size_t certainty)
    : update_interval_ms(interval_ms), last_update_time(0), certainty_factor(certainty)
{
    if (wire)
    {
        sensor.setBus(wire);
    }
}

bool VL53L1XZoneMonitor::init()
{
    if (!sensor.init())
        return false;
    sensor.startContinuous(update_interval_ms);
    return true;
}

void VL53L1XZoneMonitor::setDistanceMode(VL53L1X::DistanceMode mode)
{
    sensor.setDistanceMode(mode);
}

VL53L1X::DistanceMode VL53L1XZoneMonitor::getDistanceMode()
{
    return sensor.getDistanceMode();
}

void VL53L1XZoneMonitor::setMeasurementTimingBudget(uint32_t budget_us)
{
    sensor.setMeasurementTimingBudget(budget_us);
}

uint32_t VL53L1XZoneMonitor::getMeasurementTimingBudget()
{
    return sensor.getMeasurementTimingBudget();
}

void VL53L1XZoneMonitor::setTimeout(uint16_t timeout)
{
    sensor.setTimeout(timeout);
}

uint16_t VL53L1XZoneMonitor::getTimeout()
{
    return sensor.getTimeout();
}

void VL53L1XZoneMonitor::addZone(uint16_t min, uint16_t max, void (*onEnter)(uint16_t), void (*onExit)())
{
    zones.emplace_back(min, max, onEnter, onExit);
}

bool VL53L1XZoneMonitor::isObjectInZone(size_t zone_index)
{
    performUpdate();
    if (zone_index < zones.size())
    {
        return zones[zone_index].isObjectPresent();
    }
    return false;
}

size_t VL53L1XZoneMonitor::getZoneCount() const
{
    return zones.size();
}

ZoneObserver *VL53L1XZoneMonitor::getZone(size_t zone_index)
{
    if (zone_index < zones.size())
    {
        return &zones[zone_index];
    }
    return nullptr;
}

void VL53L1XZoneMonitor::updateZone(size_t zone_index, uint16_t min_distance, uint16_t max_distance)
{
    if (zone_index < zones.size())
    {
        if (min_distance != 0)
        {
            zones[zone_index].min_distance = min_distance;
        }

        if (max_distance != 0)
        {
            zones[zone_index].max_distance = max_distance;
        }
    }
}



void VL53L1XZoneMonitor::deleteZone(size_t zone_index)
{
    if (zone_index < zones.size())
    {
        zones.erase(zones.begin() + zone_index);
    }
}

uint16_t VL53L1XZoneMonitor::getDistance()
{
    if (sensor.dataReady())
    {
        return sensor.read(false);
    }
    return 0;
}

void VL53L1XZoneMonitor::setCertaintyFactor(size_t certainty)
{
    certainty_factor = certainty;
}

size_t VL53L1XZoneMonitor::getCertaintyFactor() const
{
    return certainty_factor;
}

void VL53L1XZoneMonitor::performUpdate()
{
    if (millis() - last_update_time >= update_interval_ms)
    {
        last_update_time = millis();
        if (sensor.dataReady())
        {
            uint16_t distance = sensor.read(false);
            for (auto &zone : zones)
            {
                zone.evaluate(distance, certainty_factor);
            }
        }
    }
}

void VL53L1XZoneMonitor::update()
{
    performUpdate();
}
