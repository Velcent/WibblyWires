// Copyright 2022 Geordie Hall. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "BlueprintConnectionDrawingPolicy.h"
#include "ConnectionDrawingPolicy.h"
#include "Verlet.h"
#include "EdGraphUtilities.h"
#include "Engine/SpringInterpolator.h"

struct FWireId
{
	FWireId(UEdGraphPin* InStartPin, UEdGraphPin* InEndPin)
	: StartPin(InStartPin)
	, EndPin(InEndPin)
	, StartPinHandle(FGraphPinHandle(InStartPin))
	, EndPinHandle(FGraphPinHandle(InEndPin))
	{
		uint32 StartHash = StartPin ? GetTypeHash(StartPin->PinId) : 0;
		uint32 EndHash = EndPin ? GetTypeHash(EndPin->PinId) : 0;
		Hash = HashCombine(StartHash, EndHash);
	}

	FORCEINLINE bool operator ==(const FWireId& Other) const
	{
        return StartPinHandle == Other.StartPinHandle && EndPinHandle == Other.EndPinHandle;
	}

	FORCEINLINE bool operator !=(const FWireId& Other) const
	{
        return StartPinHandle != Other.StartPinHandle || EndPinHandle != Other.EndPinHandle;
	}

	friend uint32 GetTypeHash(const FWireId& WireId)
	{
		return WireId.Hash;
	}

	bool IsPreviewConnector() const
	{
		return StartPin == nullptr || EndPin == nullptr;
	}

	const UEdGraphPin* GetConnectedPin() const
	{
		return StartPin == nullptr ? EndPin : StartPin;
	}

public:
	const UEdGraphPin* StartPin;
	const UEdGraphPin* EndPin;
	FGraphPinHandle StartPinHandle;
	FGraphPinHandle EndPinHandle;

private:
	uint32 Hash;
};

struct FWireState
{
	float DesiredRopeLength;
	float LerpedRopeLength;
	FVector2f DesiredRopeCenterPoint;
	FRK4SpringInterpolator<FVector> SpringCenterPoint;
	float DesiredSlackMultiplier;
	FVector2f LastStartPoint;
	FVector2f LastEndPoint;
	FLinearColor Color;
	// Simpan titik sampel kurva bezier untuk dipakai saat wire dihapus
    TArray<FVector2f> SampledPoints;

	FWireState() = default;
	FWireState(FVector2f StartPoint, FVector2f EndPoint, float SpringStiffness, float SpringDampeningRatio, float InDesiredSlackMultiplier);

	FVector2f CalculateDesiredCenterPointWithRopeLengthDelta(FVector2f StartPoint, FVector2f EndPoint, float RopeLengthDelta);
	FVector2f CalculateDesiredCenterPoint(FVector2f StartPoint, FVector2f EndPoint);
	float CalculateDesiredRopeLength(FVector2f StartPoint, FVector2f EndPoint);
	FVector2f Update(FVector2f StartPoint, FVector2f EndPoint, float DeltaTime);
};

struct FGraphState
{
	TMap<FWireId, FWireState> Wires;
	FVerletState VerletWires;

	// Tambahkan pelacakan wire yang aktif pada frame ini
    TSet<FWireId> ActiveWires;
    uint64 LastActiveFrame = 0;
};

/**
 * A drawing policy that wibbles
 */
class FWibblyConnectionDrawingPolicy : public FKismetConnectionDrawingPolicy
{
public:
	FWibblyConnectionDrawingPolicy(int32 InBackLayerID, int32 InFrontLayerID, float InZoomFactor, const FSlateRect& InClippingRect, FSlateWindowElementList& InDrawElements, UEdGraph* InGraphObj);

	struct Factory : public FGraphPanelPinConnectionFactory
	{
	public:
		virtual ~Factory() = default;

		// FGraphPanelPinConnectionFactory
		virtual class FConnectionDrawingPolicy* CreateConnectionPolicy(
			const UEdGraphSchema* Schema,
			int32 InBackLayerID,
			int32 InFrontLayerID,
			float InZoomFactor,
			const FSlateRect& InClippingRect,
			FSlateWindowElementList& InDrawElements,
			UEdGraph* InGraphObj) const override;
		// ~FGraphPanelPinConnectionFactory
	};

	// Cache variabel yang sebelumnya private di base class
    int32 CachedFrontLayerID;
    float CachedZoomFactor;
    // Voting delta pergerakan wire per frame (untuk deteksi pan/scroll view)
    TMap<FIntPoint, int32> DeltaVotes;

	virtual void DrawConnection(int32 LayerId, const FVector2f& Start, const FVector2f& End, const FConnectionParams& Params) override;

	// Override fungsi Draw untuk menangani logika penghapusan
    virtual void Draw(TMap<TSharedRef<SWidget>, FArrangedWidget>& InPinGeometries, FArrangedChildren& InArrangedNodes) override;

private:

	UEdGraph* GraphObj;
	FGraphState& GraphState;
};
