#include "WibblySettings.h"
#include "HAL/IConsoleManager.h"

void UWibblySettings::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    // Sinkronkan ke CVar setiap kali ada property yang berubah
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.Enabled")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.Enabled"))->Set(bEnableWibblyWires ? 1 : 0);
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.ThicknessMultiplier")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.ThicknessMultiplier"))->Set(ThicknessMultiplier);
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.WireLength")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.WireLength"))->Set(WireLengthHangMultiplier);
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.BounceWires")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.BounceWires"))->Set(bBounceWires ? 1 : 0);
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.WireShrinkRate")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.WireShrinkRate"))->Set(WireShrinkRate);
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.SecondsBeforeBreaking")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.SecondsBeforeBreaking"))->Set(SecondsBeforeBreaking);
    if (IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.WireFriction")))
        IConsoleManager::Get().FindConsoleVariable(TEXT("WibblyWires.WireFriction"))->Set(WireFriction);
}