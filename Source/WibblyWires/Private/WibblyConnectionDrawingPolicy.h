// Copyright 2022 Geordie Hall. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "BlueprintConnectionDrawingPolicy.h"
#include "ConnectionDrawingPolicy.h"
#include "Verlet.h"
#include "EdGraphUtilities.h"
#include "Engine/SpringInterpolator.h"
#include "Framework/Application/IInputProcessor.h"
#include "Input/Events.h"

struct FWireId
{
	FWireId()
		: StartPin(nullptr)
		, EndPin(nullptr)
		, Hash(0)
	{
	}

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

	// Helper untuk mencari pin yang valid di dalam Graph (karena UEdGraphPin bukan UObject)
	UEdGraphPin* GetStartPin(UEdGraph* Graph) const
	{
		if (!StartPinHandle.IsValid() || Graph == nullptr) return nullptr;
		for (UEdGraphNode* Node : Graph->Nodes)
		{
			if (Node && Node->NodeGuid == StartPinHandle.NodeGuid)
			{
				for (UEdGraphPin* Pin : Node->Pins)
				{
					if (Pin && Pin->PinId == StartPinHandle.PinId) return Pin;
				}
			}
		}
		return nullptr;
	}

	UEdGraphPin* GetEndPin(UEdGraph* Graph) const
	{
		if (!EndPinHandle.IsValid() || Graph == nullptr) return nullptr;
		for (UEdGraphNode* Node : Graph->Nodes)
		{
			if (Node && Node->NodeGuid == EndPinHandle.NodeGuid)
			{
				for (UEdGraphPin* Pin : Node->Pins)
				{
					if (Pin && Pin->PinId == EndPinHandle.PinId) return Pin;
				}
			}
		}
		return nullptr;
	}

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
	int32 MissingFrameCount = 0;

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

// ===== Registry geometri wire untuk alat potong Alt+Drag =====
struct FWibblyCutRegistry
{
	struct FWireGeom
	{
		FWireId Id;
		TArray<FVector2f> Points;
	};

	static FWibblyCutRegistry& Get();

	void RequestStart();
	bool ConsumeStartRequest();
	void BeginPublish(UEdGraph* InGraph);
	void AddWire(const FWireId& InId, const TArray<FVector2f>& InPoints);
	void EndPublish(const FVector2f& InGraphMousePos);
	void AccumulateIntersections(const FVector2f& InGraphMousePos);
	void FinishCut();
	void CancelCut();

	TArray<FWireGeom> Wires;
	UEdGraph* Graph = nullptr;
	double StartRequestTime = -1.0;
	bool bCutting = false;
	FVector2f CutStart = FVector2f::ZeroVector;
	TSet<FWireId> Pending;
};

// ===== Input processor: menangkap Alt+LMB drag =====
class FWibblyCutInputProcessor : public IInputProcessor
{
public:
	// UE 5.8: signature Tick yang baru (DeltaTime, SlateApp, Cursor)
	virtual void Tick(const float DeltaTime, FSlateApplication& SlateApp, TSharedRef<ICursor> Cursor) override {}

	// Stub wajib / pelengkap (tidak menelan event)
	virtual bool HandleKeyDownEvent(FSlateApplication& SlateApp, const FKeyEvent& InKeyEvent) override { return false; }
	virtual bool HandleKeyUpEvent(FSlateApplication& SlateApp, const FKeyEvent& InKeyEvent) override { return false; }
	virtual bool HandleAnalogInputEvent(FSlateApplication& SlateApp, const FAnalogInputEvent& InAnalogInputEvent) override { return false; }
	virtual bool HandleMouseMoveEvent(FSlateApplication& SlateApp, const FPointerEvent& MouseEvent) override { return false; }
	virtual bool HandleMotionDetectedEvent(FSlateApplication& SlateApp, const FMotionEvent& MotionEvent) override { return false; }

	// CATATAN: HandleMouseWheelOrGestureEvent TIDAK perlu di-override lagi di UE 5.8
	// (sudah ada implementasi default di base class yang return false) -> barisnya DIHAPUS.

	// Dua method inti fitur cut Alt+Drag (implementasi tetap di .cpp)
	virtual bool HandleMouseButtonDownEvent(FSlateApplication& SlateApp, const FPointerEvent& InEvent) override;
	virtual bool HandleMouseButtonUpEvent(FSlateApplication& SlateApp, const FPointerEvent& InEvent) override;
};