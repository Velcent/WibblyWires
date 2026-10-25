#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "WibblySettings.generated.h"

UCLASS(config = Editor, defaultconfig, meta = (DisplayName = "Wibbly Wires"))
class WIBBLYWIRES_API UWibblySettings : public UDeveloperSettings
{
    GENERATED_BODY()

public:
    UWibblySettings(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
    {
        CategoryName = TEXT("Plugins");
    }

    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "General", meta = (DisplayName = "Enable Wibbly Wires"))
    bool bEnableWibblyWires = true;

    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "Physics", meta = (DisplayName = "Thickness Multiplier", ClampMin = "0.1"))
    float ThicknessMultiplier = 1.5f;

    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "Physics", meta = (DisplayName = "Extra Wire Length (Slack)", ClampMin = "0.1"))
    float WireLengthHangMultiplier = 1.0f;

    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "Physics", meta = (DisplayName = "Enable Wire Bounce"))
    bool bBounceWires = false;

    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "Physics", meta = (DisplayName = "Wire Shrink Rate", ClampMin = "0.0"))
    float WireShrinkRate = 150.0f;

    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "Physics", meta = (DisplayName = "Seconds Before Breaking", ClampMin = "0.0"))
    float SecondsBeforeBreaking = 1.0f;
    
    UPROPERTY(config, EditAnywhere, BlueprintReadOnly, Category = "Physics", meta = (DisplayName = "Wire Friction", ClampMin = "0.5", ClampMax = "1.0"))
    float WireFriction = 0.9996f;

    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
};