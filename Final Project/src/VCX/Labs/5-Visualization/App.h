#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/5-Visualization/CaseWordCloud.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Visualization {
    class App : public Engine::IApp {
    private:
        Common::UI              _ui;
        std::size_t             _caseId = 0;
        CaseWordCloud _caseWordCloud;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseWordCloud
        };

    public:
        App();

        void OnFrame() override;
    };
}
