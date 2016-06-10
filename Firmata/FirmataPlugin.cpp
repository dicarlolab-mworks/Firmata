//
//  FirmataPlugin.cpp
//  Firmata
//
//  Created by Christopher Stawarz on 6/10/16.
//  Copyright © 2016 The MWorks Project. All rights reserved.
//

//#include "FirmataDevice.hpp"


BEGIN_NAMESPACE_MW


class FirmataPlugin : public Plugin {
    void registerComponents(boost::shared_ptr<ComponentRegistry> registry) override {
    }
};


extern "C" Plugin* getPlugin() {
    return new FirmataPlugin();
}


END_NAMESPACE_MW
