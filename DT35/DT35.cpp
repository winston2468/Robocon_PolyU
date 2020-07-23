#include "./DT35.h"

    DT35::DT35(PinName sda, PinName sci, uint8_t address1)
    {
        this->vSenor1 = new INA3221(sda, sci, address1);
    }

    DT35::DT35(PinName sda, PinName sci, uint8_t address1, uint8_t address2)
    {
        this->vSenor1 = new INA3221(sda, sci, address1);
        this->vSenor2 = new INA3221(sda, sci, address2);
    }

    DT35::DT35(PinName sda, PinName sci, uint8_t address1, uint8_t address2, uint8_t address3)
    {
        this->vSenor1 = new INA3221(sda, sci, address1);
        this->vSenor2 = new INA3221(sda, sci, address2);
        this->vSenor3 = new INA3221(sda, sci, address3);
    }

    void DT35::DT35_initialization(int INA3221_id, int channel_number)
    { 
        if(INA3221_id == 1){
            if(channel_number == 1){
                this->vSenor1->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor1->EnableChannel(1);
            }
            else if(channel_number == 2){
                this->vSenor1->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor1->EnableChannel(1);
                this->vSenor1->EnableChannel(2);
            }
            else if(channel_number == 3){
                this->vSenor1->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor1->EnableChannel(1);
                this->vSenor1->EnableChannel(2);
                this->vSenor1->EnableChannel(3);
            }
        }
        else if(INA3221_id == 2){
            if(channel_number == 1){
                this->vSenor2->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor2->EnableChannel(1);
            }
            else if(channel_number == 2){
                this->vSenor2->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor2->EnableChannel(1);
                this->vSenor2->EnableChannel(2);
            }
            else if(channel_number == 3){
                this->vSenor2->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor2->EnableChannel(1);
                this->vSenor2->EnableChannel(2);
                this->vSenor2->EnableChannel(3);
            }
        }
        else if(INA3221_id == 3){
            if(channel_number == 1){
                this->vSenor3->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor3->EnableChannel(1);
            }
            else if(channel_number == 2){
                this->vSenor3->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor3->EnableChannel(1);
                this->vSenor3->EnableChannel(2);
            }
            else if(channel_number == 3){
                this->vSenor3->SetMode(INA3221_MODE_BUS_CONTINUOUS);
                this->vSenor3->EnableChannel(1);
                this->vSenor3->EnableChannel(2);
                this->vSenor3->EnableChannel(3);
            }
        }
    }

    int DT35::getManufacturerID(int INA3221_id)
    {
        int temp = 0;
        if(INA3221_id == 1)
        {
            temp = this->vSenor1->GetManufacturerID();
        }
        else if(INA3221_id == 2)
        {
            temp = this->vSenor2->GetManufacturerID();
        }
        else if(INA3221_id == 3)
        {
            temp = this->vSenor3->GetManufacturerID();
        }
        return temp;
    }

    int DT35::getDieID(int INA3221_id)
    {
        int temp = 0;
        if(INA3221_id == 1)
        {
            temp = this->vSenor1->GetDieID();
        }
        else if(INA3221_id == 2)
        {
            temp = this->vSenor2->GetDieID();
        }
        else if(INA3221_id == 3)
        {
            temp = this->vSenor3->GetDieID();
        }
        return temp;
    }

    int DT35::getConfiguration(int INA3221_id)
    {
        int temp = 0;
        if(INA3221_id == 1)
        {
            temp = this->vSenor1->GetConfiguration();
        }
        else if(INA3221_id == 2)
        {
            temp = this->vSenor2->GetConfiguration();
        }
        else if(INA3221_id == 3)
        {
            temp = this->vSenor2->GetConfiguration();
        }
        return temp;
    }

    int DT35::getBusVoltage(int INA3221_id, int channel_id)
    {
        int temp = 0;
        if(INA3221_id == 1)
        {
            temp = this->vSenor1->GetBusVoltage(channel_id) * 1000; //V to mV
        }
        else if(INA3221_id == 2)
        {
            temp = this->vSenor2->GetBusVoltage(channel_id) * 1000; //V to mV
        }
        else if(INA3221_id == 3)
        {
            temp = this->vSenor3->GetBusVoltage(channel_id) * 1000; //V to mV
        }
        return temp;
    }