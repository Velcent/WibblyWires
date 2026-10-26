// Copyright 2022 Geordie Hall. All rights reserved.


#include "WibblyConnectionDrawingPolicy.h"

#include "EdGraphSchema_K2.h"
#include "Verlet.h"
#include "WibblySettings.h"
#include "Engine/SpringInterpolator.h"
#include "K2Node_Knot.h"
#include "ScopedTransaction.h"
#include "EdGraph/EdGraphSchema.h"
#include "HAL/PlatformTime.h"
#include "Widgets/SWindow.h"
#include "InputCoreTypes.h"

// For each graph guid, store a map from wire id to wire state
static TMap<FGuid, FGraphState> GraphStates;

static int32 EnableWibblyWires = 1;
FAutoConsoleVariableRef CVarEnableWibblyWires(
	TEXT("WibblyWires.Enabled"),
	EnableWibblyWires,
	TEXT("Whether BP wires should be Wibbly."));

static float ThicknessMultiplier = 1.5f;
FAutoConsoleVariableRef CVarThicknessMultiplier(
	TEXT("WibblyWires.ThicknessMultiplier"),
	ThicknessMultiplier,
	TEXT("How much thicker to draw the wire lines."));

static int32 BounceWires = 0;
FAutoConsoleVariableRef CVarBounceWires(
	TEXT("WibblyWires.BounceWires"),
	BounceWires,
	TEXT("Whether wires have some bounce when they extend too far")
);

static float RopeLengthHangMultiplier = 1.f;
FAutoConsoleVariableRef CVarWireLength(
	TEXT("WibblyWires.WireLength"),
	RopeLengthHangMultiplier,
	TEXT("How much extra length should wires have")
);

float WireShrinkRate = 150.f;
FAutoConsoleVariableRef CVarWireShrinkRate(
	TEXT("WibblyWires.WireShrinkRate"),
	WireShrinkRate,
	TEXT("How quickly should wires get sucked back into their nodes after having been cut")
);

float SecondsBeforeBreaking = 1.f;
FAutoConsoleVariableRef CVarSecondsBeforeBreaking(
	TEXT("WibblyWires.SecondsBeforeBreaking"),
	SecondsBeforeBreaking,
	TEXT("How many seconds should cut wires dangle before detaching from their nodes and falling")
);

float WireFriction = 0.9996f;
FAutoConsoleVariableRef CVarWireFriction(
	TEXT("WibblyWires.WireFriction"),
	WireFriction,
	TEXT("Friction multiplier for velocities, should be very close to 1.")
);

FAutoConsoleCommand CVarResetWireStates(
	TEXT("WibblyWires.ResetWireStates"),
	TEXT("Resets wire states so that they're reinitialized with latest defaults etc."),
	FConsoleCommandDelegate::CreateLambda([]()
	{
		GraphStates.Reset();
	})
);

// True if segment A0-A1 intersects segment B0-B1
bool SegmentsIntersect2D(const FVector2f& A0, const FVector2f& A1, const FVector2f& B0, const FVector2f& B1)
{
	const FVector2f dA = A1 - A0;
	const FVector2f dB = B1 - B0;
	const FVector2f d  = B0 - A0;

	const float Den = dA.X * dB.Y - dA.Y * dB.X;
	if (FMath::IsNearlyZero(Den))
	{
		return false; // parallel/collinear
	}

	const float T = (d.X * dB.Y - d.Y * dB.X) / Den;
	const float U = (d.X * dA.Y - d.Y * dA.X) / Den;
	return T >= 0.0f && T <= 1.0f && U >= 0.0f && U <= 1.0f;
}

FWireState::FWireState(FVector2f StartPoint, FVector2f EndPoint, float SpringStiffness, float SpringDampeningRatio, float InDesiredSlackMultiplier)
{
	LastStartPoint = StartPoint;
	LastEndPoint = EndPoint;
	DesiredSlackMultiplier = InDesiredSlackMultiplier;

	// Snap to the desired rope length
	DesiredRopeLength = CalculateDesiredRopeLength(StartPoint, EndPoint);
	LerpedRopeLength = DesiredRopeLength * 1.1f; // Start off a little off from desired so there's an initial bounce

	// Snap to the desired center point
	float LengthDelta = LerpedRopeLength - (EndPoint - StartPoint).Size();
	DesiredRopeCenterPoint = CalculateDesiredCenterPointWithRopeLengthDelta(StartPoint, EndPoint, LengthDelta);
	SpringCenterPoint.SetSpringConstants(SpringStiffness, SpringDampeningRatio);
	SpringCenterPoint.Reset(FVector(DesiredRopeCenterPoint.X, DesiredRopeCenterPoint.Y, 0.f));
}

FVector2f FWireState::CalculateDesiredCenterPointWithRopeLengthDelta(FVector2f StartPoint, FVector2f EndPoint, float RopeLengthDelta)
{
	FVector2f Center = CalculateDesiredCenterPoint(StartPoint, EndPoint);
	Center.Y += RopeLengthDelta * RopeLengthHangMultiplier;
	return Center;
}

FVector2f FWireState::CalculateDesiredCenterPoint(FVector2f StartPoint, FVector2f EndPoint)
{
	if (StartPoint.X > EndPoint.X)
	{
		Swap(StartPoint, EndPoint);
	}

	FVector2f Delta = EndPoint - StartPoint;
	FVector2f Direction = Delta.GetSafeNormal();
	FVector2f UpDirection(0.f, 1.f);
	float DotWithUp = Direction | UpDirection;
	DotWithUp = FMath::Pow(FMath::Abs(DotWithUp), 2.f) * FMath::Sign(DotWithUp);
	float NormalizedDotWithUp = DotWithUp * 0.5f + 0.5f;
	float CenterX = FMath::Lerp(StartPoint.X, EndPoint.X, NormalizedDotWithUp);
	// This won't be quite the same as deriving a CenterY from the real CenterX, but we'll see how it looks cause avoids some trig
	FVector2f Center = FMath::Lerp(StartPoint, EndPoint, NormalizedDotWithUp);
	return Center;
}

float FWireState::CalculateDesiredRopeLength(FVector2f StartPoint, FVector2f EndPoint)
{
	FVector2f Delta = EndPoint - StartPoint;
	float TightRopeLength = Delta.Size();
	return TightRopeLength * DesiredSlackMultiplier;
}

FVector2f FWireState::Update(FVector2f StartPoint, FVector2f EndPoint, float DeltaTime)
{
	LastStartPoint = StartPoint;
	LastEndPoint = EndPoint;

	// Ensure start point is always the left-most point so we can make some assumptions with our math
	if (StartPoint.X > EndPoint.X)
	{
		Swap(StartPoint, EndPoint);
	}

	FVector2f Delta = EndPoint - StartPoint;

	float TightRopeLength = Delta.Size();

	DesiredRopeLength = TightRopeLength * DesiredSlackMultiplier;
	LerpedRopeLength = FMath::Max(TightRopeLength, FMath::Lerp(LerpedRopeLength, DesiredRopeLength, DeltaTime * 20.f));
	float LengthDelta = LerpedRopeLength - TightRopeLength;

	DesiredRopeCenterPoint = CalculateDesiredCenterPointWithRopeLengthDelta(StartPoint, EndPoint, LengthDelta);

	// Calculate desired center point
	FVector2f LerpedCenterPoint = FVector2f(FVector2D(SpringCenterPoint.Update(FVector(DesiredRopeCenterPoint.X, DesiredRopeCenterPoint.Y , 0.f), DeltaTime)));

	FVector2f Velocity = FVector2f(FVector2D(SpringCenterPoint.GetVelocity()));
	if (BounceWires && LerpedCenterPoint.Y > DesiredRopeCenterPoint.Y && Velocity.Y > 0.1f)
	{
		Velocity.Y = FMath::Abs(Velocity.Y) * -0.9f;
		SpringCenterPoint.SetVelocity(FVector(Velocity.X, Velocity.Y, 0.f));
	}

	return LerpedCenterPoint;
}

FConnectionDrawingPolicy* FWibblyConnectionDrawingPolicy::Factory::CreateConnectionPolicy(const UEdGraphSchema* Schema, int32 InBackLayerID, int32 InFrontLayerID, float InZoomFactor, const FSlateRect& InClippingRect, FSlateWindowElementList& InDrawElements, UEdGraph* InGraphObj) const
{
	if (!GetDefault<UWibblySettings>()->bEnableWibblyWires)
	{
		return nullptr;
	}
	
	if (EnableWibblyWires)
	{
		if (Schema->IsA(UEdGraphSchema_K2::StaticClass()))
		{
			return new FWibblyConnectionDrawingPolicy(InBackLayerID, InFrontLayerID, InZoomFactor, InClippingRect, InDrawElements, InGraphObj);
		}
	}
	else
	{
		// Release our memory if not even enabled
		GraphStates.Empty();
	}

	return nullptr;
}

namespace FRK4SpringInterpolatorUtils
{
	static FORCEINLINE bool IsValidValue(FVector2f Value, float MaxAbsoluteValue = RK4_SPRING_INTERPOLATOR_MAX_VALUE)
	{
		return Value.GetAbsMax() < MaxAbsoluteValue;
	}

	static FORCEINLINE bool IsValidValue(FVector3f Value, float MaxAbsoluteValue = RK4_SPRING_INTERPOLATOR_MAX_VALUE)
	{
		return Value.GetAbsMax() < MaxAbsoluteValue;
	}

#if ENGINE_MAJOR_VERSION >= 5 && ENGINE_MINOR_VERSION >= 1
	static FORCEINLINE bool AreEqual(FVector2f A, FVector2f B, float ErrorTolerance = UE_KINDA_SMALL_NUMBER)
	{
		return A.Equals(B, ErrorTolerance);
	}

	static FORCEINLINE bool AreEqual(FVector3f A, FVector3f B, float ErrorTolerance = UE_KINDA_SMALL_NUMBER)
	{
		return A.Equals(B, ErrorTolerance);
	}
#else
	static FORCEINLINE bool AreEqual(FVector2f A, FVector2f B, float ErrorTolerance = KINDA_SMALL_NUMBER)
	{
		return A.Equals(B, ErrorTolerance);
	}
#endif

}

FWibblyConnectionDrawingPolicy::FWibblyConnectionDrawingPolicy(int32 InBackLayerID, int32 InFrontLayerID, float InZoomFactor, const FSlateRect& InClippingRect, FSlateWindowElementList& InDrawElements, UEdGraph* InGraphObj)
	: FKismetConnectionDrawingPolicy(InBackLayerID, InFrontLayerID, InZoomFactor, InClippingRect, InDrawElements, InGraphObj)
	, GraphObj(InGraphObj)
	, GraphState(GraphStates.FindOrAdd(InGraphObj->GraphGuid))
{
	// Simpan nilai Layer ID dan Zoom untuk digunakan nanti
    CachedFrontLayerID = InFrontLayerID;
    CachedZoomFactor = InZoomFactor;
}

// Berapa frame berturut-turut wire harus "hilang" sebelum boleh dianggap cut.
// Mencegah frame transien (rekonstruksi graph saat undo/redo/compile) memicu animasi.
static constexpr int32 RequiredMissingFramesBeforeCut = 2;
// Batas cleanup state preview connector yang sudah basi
static constexpr int32 PreviewConnectorMaxMissingFrames = 60;
// Cut asli terjadi satu wire per aksi. Jika lebih dari ini dalam 1 frame,
// pasti graph sedang direstrukturisasi (undo/redo/paste/compile) -> jangan animasikan.
static constexpr int32 MaxCutChainsPerFrame = 3;

// Dua pin masih terhubung lewat satu atau lebih reroute node?
// Double click wire = wire DIBELAH (reroute disisipkan), bukan dipotong.
static bool ArePinsConnectedThroughReroutes(UEdGraphPin* StartPin, UEdGraphPin* EndPin)
{
	if (StartPin == nullptr || EndPin == nullptr)
	{
		return false;
	}

	if (StartPin->LinkedTo.Contains(EndPin))
	{
		return true;
	}

	TSet<const UEdGraphNode*> VisitedNodes;
	TArray<const UEdGraphPin*> PinStack;

	for (UEdGraphPin* LinkedPin : StartPin->LinkedTo)
	{
		PinStack.Add(LinkedPin);
	}

	while (PinStack.Num() > 0)
	{
		const UEdGraphPin* CurrentPin = PinStack.Pop(EAllowShrinking::No);
		if (CurrentPin == nullptr || CurrentPin == EndPin)
		{
			return CurrentPin == EndPin;
		}

		const UEdGraphNode* OwnerNode = CurrentPin->GetOwningNode();
		if (OwnerNode == nullptr || VisitedNodes.Contains(OwnerNode))
		{
			continue;
		}
		VisitedNodes.Add(OwnerNode);

		// Hanya boleh "menembus" reroute node, node lain bukan jalur split
		if (!OwnerNode->IsA(UK2Node_Knot::StaticClass()))
		{
			continue;
		}

		for (UEdGraphPin* ReroutePin : OwnerNode->Pins)
		{
			for (UEdGraphPin* NextPin : ReroutePin->LinkedTo)
			{
				if (NextPin == EndPin)
				{
					return true;
				}
				PinStack.Add(NextPin);
			}
		}
	}

	return false;
}

// ---------- Helper geometri ----------
static float Cross2D(const FVector2f& A, const FVector2f& B) { return A.X * B.Y - A.Y * B.X; }

static bool SegSegIntersect(const FVector2f& P1, const FVector2f& P2, const FVector2f& P3, const FVector2f& P4)
{
	const FVector2f D1 = P2 - P1;
	const FVector2f D2 = P4 - P3;
	const float Den = Cross2D(D1, D2);
	if (FMath::Abs(Den) < 1e-8f) return false;
	const FVector2f D3 = P3 - P1;
	const float T = Cross2D(D3, D2) / Den;
	const float U = Cross2D(D3, D1) / Den;
	return (T >= 0.f && T <= 1.f && U >= 0.f && U <= 1.f);
}

static bool SegmentIntersectsPolyline(const FVector2f& A, const FVector2f& B, const TArray<FVector2f>& P)
{
	for (int32 i = 0; i + 1 < P.Num(); ++i)
	{
		if (SegSegIntersect(A, B, P[i], P[i + 1])) return true;
	}
	return false;
}

// ---------- FWibblyCutRegistry ----------
FWibblyCutRegistry& FWibblyCutRegistry::Get()
{
	static FWibblyCutRegistry Instance;
	return Instance;
}

void FWibblyCutRegistry::RequestStart()
{
	StartRequestTime = FPlatformTime::Seconds();
	// Paksa panel repaint agar Draw() berjalan dan bisa mengambil titik mulai
	if (FSlateApplication::IsInitialized())
	{
		TSharedPtr<SWindow> Window = FSlateApplication::Get().GetActiveTopLevelWindow();
		if (Window.IsValid()) { Window->Invalidate(EInvalidateWidget::Paint); }
	}
}

bool FWibblyCutRegistry::ConsumeStartRequest()
{
	if (StartRequestTime < 0.0) return false;
	const double Age = FPlatformTime::Seconds() - StartRequestTime;
	StartRequestTime = -1.0;
	return (Age < 0.1);
}

void FWibblyCutRegistry::BeginPublish(UEdGraph* InGraph)
{
	Graph = InGraph;
	Wires.Reset();
}

void FWibblyCutRegistry::AddWire(const FWireId& InId, const TArray<FVector2f>& InPoints)
{
	FWireGeom& G = Wires.AddDefaulted_GetRef();
	G.Id = InId;
	G.Points = InPoints;
}

void FWibblyCutRegistry::EndPublish(const FVector2f& InGraphMousePos) { }

void FWibblyCutRegistry::AccumulateIntersections(const FVector2f& InGraphMousePos)
{
	for (const FWireGeom& G : Wires)
	{
		if (SegmentIntersectsPolyline(CutStart, InGraphMousePos, G.Points))
		{
			Pending.Add(G.Id);
		}
	}
}

void FWibblyCutRegistry::FinishCut()
{
	bCutting = false;
	if (Graph == nullptr || Pending.Num() == 0) { Pending.Reset(); return; }

	FScopedTransaction Transaction(NSLOCTEXT("WibblyWires", "CutWibblyWires", "Cut Wires"));
	Graph->Modify();
	for (const FWireId& Id : Pending)
	{
		UEdGraphPin* A = Id.GetStartPin(Graph);
		UEdGraphPin* B = Id.GetEndPin(Graph);
		if (A && B)
		{
			A->Modify();
			B->Modify();
			Graph->GetSchema()->BreakSinglePinLink(A, B);
		}
	}
	Graph->NotifyGraphChanged(); // wire hilang -> animasi potong Verlet otomatis jalan
	Pending.Reset();
}

void FWibblyCutRegistry::CancelCut()
{
	bCutting = false;
	Pending.Reset();
}

// ---------- FWibblyCutInputProcessor ----------
bool FWibblyCutInputProcessor::HandleMouseButtonDownEvent(FSlateApplication& SlateApp, const FPointerEvent& InEvent)
{
	if (InEvent.GetEffectingButton() == EKeys::LeftMouseButton && InEvent.IsAltDown())
	{
		FWibblyCutRegistry::Get().RequestStart();
	}
	return false; // jangan ditelan: garis putus-putus bawaan engine tetap tampil
}

bool FWibblyCutInputProcessor::HandleMouseButtonUpEvent(FSlateApplication& SlateApp, const FPointerEvent& InEvent)
{
	if (InEvent.GetEffectingButton() == EKeys::LeftMouseButton)
	{
		FWibblyCutRegistry& Reg = FWibblyCutRegistry::Get();
		if (Reg.bCutting)
		{
			if (InEvent.IsAltDown()) { Reg.FinishCut(); }
			else { Reg.CancelCut(); }
		}
	}
	return false;
}

void FWibblyConnectionDrawingPolicy::Draw(TMap<TSharedRef<SWidget>, FArrangedWidget>& InPinGeometries, FArrangedChildren& InArrangedNodes)
{
	// 1. Reset tracking per-frame (DeltaVotes JUGA wajib di-reset, dulu menumpuk selamanya)
	GraphState.ActiveWires.Reset();
	DeltaVotes.Reset();

    // 2. Panggil fungsi bawaan engine untuk menggambar wire normal
    FKismetConnectionDrawingPolicy::Draw(InPinGeometries, InArrangedNodes);
	
    // 2.5. Translate potongan wire yang ter-cut agar mengikuti pan/scroll view.
    //      Ambil delta mayoritas dari voting: saat pan SEMUA wire bergerak seragam,
    //      saat node di-drag hanya sebagian wire bergerak (mayoritas tetap 0).
    {
        FIntPoint BestKey(0, 0);
        int32 BestVotes = 1; // butuh minimal 2 suara biar tidak salah tebak
        for (const auto& Vote : DeltaVotes)
        {
            if (Vote.Value > BestVotes)
            {
                BestVotes = Vote.Value;
                BestKey = Vote.Key;
            }
        }

        // Seri dengan delta 0 -> anggap tidak ada pan (mis. lagi drag node)
        const int32 ZeroVotes = DeltaVotes.FindRef(FIntPoint(0, 0));
        if (ZeroVotes >= BestVotes)
        {
            BestKey = FIntPoint(0, 0);
        }

        if (BestKey.X != 0 || BestKey.Y != 0)
        {
            GraphState.VerletWires.TranslateVerletChains(FVector2f((float)BestKey.X, (float)BestKey.Y));
        }
    }

    // 3. Identifikasi wire yang dihapus (ada di Wires tapi tidak dipanggil di frame ini)
	TArray<FWireId> DeletedWires;   // -> animasi cut
	TArray<FWireId> ObsoleteWires;  // -> buang state diam-diam, TANPA animasi
	for (auto& Pair : GraphState.Wires)
    {
		FWireState& State = Pair.Value;

        if (GraphState.ActiveWires.Contains(Pair.Key))
        {
			State.MissingFrameCount = 0;
            continue; // masih digambar frame ini -> wire masih hidup
        }

		// Grace period: wire harus hilang beberapa frame berturut-turut.
		// Frame transien saat undo/redo/compile tidak lagi memicu cut.
		State.MissingFrameCount++;
		if (State.MissingFrameCount < RequiredMissingFramesBeforeCut)
		{
			continue;
		}

		// Preview connector: tidak pernah memicu animasi cut (fix lama dipertahankan),
		// tapi state-nya dibersihkan setelah drag selesai biar tidak bocor.
		if (Pair.Key.IsPreviewConnector())
		{
			if (State.MissingFrameCount > PreviewConnectorMaxMissingFrames)
			{
				ObsoleteWires.Add(Pair.Key);
			}
			continue;
		}

		// FIX BUG UNDO: Cari pin asli di graph menggunakan Handle. 
		// Jika hasilnya nullptr, berarti node/pin sudah di-destroy.
		UEdGraphPin* StartPin = Pair.Key.GetStartPin(GraphObj);
		UEdGraphPin* EndPin   = Pair.Key.GetEndPin(GraphObj);

		if (StartPin == nullptr || EndPin == nullptr)
		{
			ObsoleteWires.Add(Pair.Key); // pin/node lenyap -> buang state, tanpa animasi
			continue;
		}

		// Masih terhubung langsung -> hanya di-cull (off-screen / node collapse)
		if (StartPin->LinkedTo.Contains(EndPin))
		{
			continue;
		}

		// FIX BUG DOUBLE CLICK: koneksi masih hidup lewat reroute node
		// => wire di-SPLIT, bukan di-CUT. Buang state lama tanpa animasi.
		if (ArePinsConnectedThroughReroutes(StartPin, EndPin))
		{
			ObsoleteWires.Add(Pair.Key);
			continue;
		}

        // Wire asli yang koneksinya benar-benar putus / node dihapus -> animasi cut
        DeletedWires.Add(Pair.Key);
    }

	// Safety valve: user cuma bisa memotong 1 wire per aksi. Kalau dalam satu frame
	// ada banyak wire "terhapus", itu restrukturisasi graph (undo/redo/paste/compile)
	// -> batalkan semua animasi supaya tidak terspam.
	if (DeletedWires.Num() > MaxCutChainsPerFrame)
	{
		ObsoleteWires.Append(DeletedWires);
		DeletedWires.Reset();
	}

	for (const FWireId& WireId : ObsoleteWires)
	{
		GraphState.Wires.Remove(WireId);
	}
	
    static const float MaxDeltaTime = 1.f / 30.f;
    const float DeltaTime = FMath::Min(FSlateApplication::Get().GetDeltaTime(), MaxDeltaTime);

    // 4. Buat Verlet chain dari wire yang dihapus — TERPOTONG JADI 2 DI CURSOR
    for (const FWireId& WireId : DeletedWires)
    {
        const FWireState& WireState = GraphState.Wires[WireId];
        if (WireState.SampledPoints.Num() >= 2)
        {
            // Cari index titik terdekat dengan posisi cursor
            int32 SplitIndex = 0;
            float ClosestDistSq = FLT_MAX;
            for (int32 i = 0; i < WireState.SampledPoints.Num(); ++i)
            {
                float DistSq = FVector2f::DistSquared(WireState.SampledPoints[i], AbsoluteMousePosition);
                if (DistSq < ClosestDistSq)
                {
                    ClosestDistSq = DistSq;
                    SplitIndex = i;
                }
            }

            // Clamp supaya kedua potongan tetap punya minimal 2 titik
            SplitIndex = FMath::Clamp(SplitIndex, 1, WireState.SampledPoints.Num() - 2);

            // Helper untuk membuat chain dari rentang titik tertentu
            // bReverse = true  -> titik ditambahkan dari EndIdx mundur ke StartIdx
            auto CreateChainFromRange = [&](int32 StartIdx, int32 EndIdx, bool bReverse)
            {
                if (EndIdx - StartIdx < 1) return; // butuh minimal 2 titik
                FVerletChain Chain(WireState.Color, 3.0f);

                // Kumpulkan index sesuai arah
                TArray<int32> Indices;
                if (bReverse)
                {
                    for (int32 i = EndIdx; i >= StartIdx; --i) Indices.Add(i);
                }
                else
                {
                    for (int32 i = StartIdx; i <= EndIdx; ++i) Indices.Add(i);
                }

                for (int32 k = 0; k < Indices.Num(); ++k)
                {
                    // Index 0 adalah ujung di PIN -> kita PIN supaya menggantung,
                    // bukan jatuh bebas. Sisanya bebas.
                    bool bIsPin = (k == 0);
                    Chain.AddToChain(WireState.SampledPoints[Indices[k]], bIsPin);
                }

                // JANGAN langsung SetAllPinned(false) / bHasBroken = true.
                // Biarkan menggantung dulu; nanti lepas sendiri setelah SecondsBeforeBreaking.
                GraphState.VerletWires.AddChain(MoveTemp(Chain));
            };

            // Potongan kiri: node awal di index 0 -> menggantung di node awal
            CreateChainFromRange(0, SplitIndex, false);
            // Potongan kanan: dibalik agar node akhir di index 0 -> menggantung di node akhir
            CreateChainFromRange(SplitIndex, WireState.SampledPoints.Num() - 1, true);
        }
        GraphState.Wires.Remove(WireId);
    }

    // 5. Update simulasi fisika
    GraphState.VerletWires.UpdateVerletChains(DeltaTime);

    // 6. Paksa viewport untuk redraw jika ada animasi tali yang jatuh
    if (GraphState.VerletWires.HasActiveChains())
    {
        TSharedPtr<SWindow> ActiveWindow = FSlateApplication::Get().GetActiveTopLevelWindow();
        if (ActiveWindow.IsValid())
        {
            ActiveWindow->Invalidate(EInvalidateWidget::Paint);
        }
    }

    // 7. Gambar simulasi tali yang putus
    GraphState.VerletWires.RenderVerletChains(DrawElementsList, CachedFrontLayerID, CachedZoomFactor);

	// ===== Integrasi alat potong Alt+Drag =====
	{
		FWibblyCutRegistry& Reg = FWibblyCutRegistry::Get();

		if (Reg.ConsumeStartRequest())
		{
			Reg.bCutting = true;
			Reg.CutStart = AbsoluteMousePosition;
			Reg.Pending.Reset();
		}

		if (Reg.bCutting)
		{
			Reg.AccumulateIntersections(AbsoluteMousePosition);
		}

		// Publikasikan geometri wire frame ini untuk uji perpotongan
		Reg.BeginPublish(GraphObj);
		for (const auto& Pair : GraphState.Wires)
		{
			if (!Pair.Key.IsPreviewConnector() && Pair.Value.SampledPoints.Num() >= 2)
			{
				Reg.AddWire(Pair.Key, Pair.Value.SampledPoints);
			}
		}
		Reg.EndPublish(AbsoluteMousePosition);
	}
}

void FWibblyConnectionDrawingPolicy::DrawConnection(int32 LayerId, const FVector2f& Start, const FVector2f& End, const FConnectionParams& Params)
{
	const FVector2f& P0 = Start;
	const FVector2f& P1 = End;

	const float DefaultStiffness = 100.f;
    const float DefaultDampeningRatio = 0.4f;

	float WireThickness = Params.WireThickness * ThicknessMultiplier * FSlateApplication::Get().GetApplicationScale() * ZoomFactor;

    const FWireId WireId(Params.AssociatedPin1, Params.AssociatedPin2);
	GraphState.ActiveWires.Add(WireId); // Tandai wire sebagai aktif pada frame ini
    FWireState* WireState = GraphState.Wires.Find(WireId);
	const bool bWireExisted = (WireState != nullptr);

	// Create a new wire if needed
    if (!WireState)
    {
		bool bIsPreviewConnector = Params.AssociatedPin1 == nullptr || Params.AssociatedPin2 == nullptr;
		float StiffnessVariance = FMath::FRandRange(0.3f, 1.5f);
		float DampeningVariance = FMath::FRandRange(0.7f, 1.2f);
		float SlackMultiplier = 1.3f + FMath::FRandRange(0.f, 0.3f);
		float Stiffness = DefaultStiffness * StiffnessVariance + (bIsPreviewConnector ? 0.3f : 0.f);
		float DampeningRatio = FMath::Clamp(DefaultDampeningRatio * DampeningVariance, 0.3f, 0.9f);
		FWireState NewWireState(Start, End, Stiffness, DampeningRatio, SlackMultiplier);
		NewWireState.Color = Params.WireColor;

		for (const auto& ExistingState : GraphState.Wires)
		{
			if (!ExistingState.Key.IsPreviewConnector())
			{
				continue;
			}

			const UEdGraphPin* ConnectedPin = ExistingState.Key.GetConnectedPin();
			if (ConnectedPin != Params.AssociatedPin1 && ConnectedPin != Params.AssociatedPin2)
			{
				continue;
			}

    		const float DistThresholdSqr = 30.f * 30.f;
			if (FVector2f::DistSquared(ExistingState.Value.LastStartPoint, Start) < DistThresholdSqr && FVector2f::DistSquared(ExistingState.Value.LastEndPoint, End) < DistThresholdSqr)
			{
				// Inherit our initial state from this existing thing, since it was probably a preview connector that got connected
				NewWireState = ExistingState.Value;
			}
		}

		WireState = &GraphState.Wires.Add(WireId, MoveTemp(NewWireState));
    }
    // VOTING DELTA: sebelum Update() menimpa LastStartPoint, catat seberapa jauh
    // wire ini berpindah sejak frame lalu. Saat pan, semua wire pindah seragam.
    if (bWireExisted && Params.AssociatedPin1 != nullptr && Params.AssociatedPin2 != nullptr)
    {
        const FVector2f Delta = Start - WireState->LastStartPoint;
        DeltaVotes.FindOrAdd(FIntPoint(FMath::RoundToInt(Delta.X), FMath::RoundToInt(Delta.Y))) += 1;
    }
	// Clamp our tick rate to 30fps to avoid editor hitches hiding our animations, we'd rather they just pause
	static const float MaxDeltaTime = 1.f / 30.f;
	const float DeltaTime = FMath::Min(FSlateApplication::Get().GetDeltaTime(), MaxDeltaTime);
    FVector2f CenterPoint = WireState->Update(Start, End, DeltaTime);

	// Don't need these anymore!
	// const FVector2f SplineTangent = ComputeSplineTangent(P0, P1);
	// const FVector2f P0Tangent = (Params.StartDirection == EGPD_Output) ? SplineTangent : -SplineTangent;
	// const FVector2f P1Tangent = (Params.EndDirection == EGPD_Input) ? SplineTangent : -SplineTangent;

	// Magic number to get more of a bend
	const FVector2f P0Tangent = (CenterPoint - P0) * 1.3f;
	const FVector2f P1Tangent = (P1 - CenterPoint) * 1.3f;

#if ENGINE_MAJOR_VERSION > 5 || (ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION >= 7)
	// --- Alt+Drag (slice) support ---------------------------------------------------
	// We fully override DrawConnection() and never call the base implementation, so the
	// engine's slice-line vs spline intersection test (which lives inside
	// FConnectionDrawingPolicy::DrawConnection since UE5.7) never sees our wibbly curve.
	// Replicate that test here against the actual wibbly spline so cutting works.
	if (SliceLine.IsValid() && Params.AssociatedPin1 != nullptr && Params.AssociatedPin2 != nullptr)
	{
		const FVector2f SliceStart = SliceLine.StartPoint;
		const FVector2f SliceEnd   = SliceLine.EndPoint;

		// Cheap reject: control-point bounds of the cubic vs bounds of the slice segment
		const float MaximumTangentContribution = 1.f / 3.f;
		FBox2f WireBounds(ForceInit);
		WireBounds += FVector2f(P0);
		WireBounds += FVector2f(P0 + MaximumTangentContribution * P0Tangent);
		WireBounds += FVector2f(P1);
		WireBounds += FVector2f(P1 - MaximumTangentContribution * P1Tangent);

		FBox2f SliceBounds(ForceInit);
		SliceBounds += SliceStart;
		SliceBounds += SliceEnd;

		if (WireBounds.Intersect(SliceBounds))
		{
			// Sample the drawn cubic spline and test each chord against the slice segment
			const int32 NumStepsToTest = 16;
			const float StepInterval = 1.0f / (float)NumStepsToTest;
			FVector2f Point1 = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, 0.0f);
			bool bSliceIntersectsWire = false;
			for (int32 StepIndex = 0; StepIndex < NumStepsToTest && !bSliceIntersectsWire; ++StepIndex)
			{
				const FVector2f Point2 = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, (StepIndex + 1) * StepInterval);
				bSliceIntersectsWire = SegmentsIntersect2D(SliceStart, SliceEnd, Point1, Point2);
				Point1 = Point2;
			}

			if (bSliceIntersectsWire)
			{
				ConnectionsIntersectingSliceLine.Emplace(Params.AssociatedPin1, Params.AssociatedPin2);
			}
		}
	}
#endif
    // Setelah menghitung CenterPoint, P0Tangent, P1Tangent, tambahkan sampling:
    WireState->SampledPoints.Reset();
    const int32 NumSegments = 20; // Banyaknya segmen tali
    for (int32 i = 0; i <= NumSegments; ++i)
    {
        float Alpha = (float)i / (float)NumSegments;
        FVector2f Point = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, Alpha);
        WireState->SampledPoints.Add(Point);
    }

	if (Settings->bTreatSplinesLikePins)
	{
		// Distance to consider as an overlap
		const float QueryDistanceTriggerThresholdSquared = FMath::Square(Settings->SplineHoverTolerance + WireThickness * 0.5f);

		// Distance to pass the bounding box cull test. This is used for the bCloseToSpline output that can be used as a
		// dead zone to avoid mistakes caused by missing a double-click on a connection.
		const float QueryDistanceForCloseSquared = FMath::Square(FMath::Sqrt(QueryDistanceTriggerThresholdSquared) + Settings->SplineCloseTolerance);

		bool bCloseToSpline = false;
		{
			// The curve will include the endpoints but can extend out of a tight bounds because of the tangents
			// P0Tangent coefficient maximizes to 4/27 at a=1/3, and P1Tangent minimizes to -4/27 at a=2/3.
			// const float MaximumTangentContribution = 4.0f / 27.0f;

			// Note (Geordie): If we don't use the engine's tangent limits then need to use full control-point bounds
			const float MaximumTangentContribution = 1.f / 3.f;
			FBox2f Bounds(ForceInit);

			Bounds += FVector2f(P0);
			Bounds += FVector2f(P0 + MaximumTangentContribution * P0Tangent);
			Bounds += FVector2f(P1);
			Bounds += FVector2f(P1 - MaximumTangentContribution * P1Tangent);

			bCloseToSpline = Bounds.ComputeSquaredDistanceToPoint(AbsoluteMousePosition) < QueryDistanceForCloseSquared;

			// Draw the bounding box for debugging
#if 0
#define DrawSpaceLine(Point1, Point2, DebugWireColor) {const FVector2f FakeTangent = (Point2 - Point1).GetSafeNormal(); FSlateDrawElement::MakeDrawSpaceSpline(DrawElementsList, LayerId, Point1, FakeTangent, Point2, FakeTangent, ClippingRect, 1.0f, ESlateDrawEffect::None, DebugWireColor); }

			if (bCloseToSpline)
			{
				const FLinearColor BoundsWireColor = bCloseToSpline ? FLinearColor::Green : FLinearColor::White;

				FVector2f TL = Bounds.Min;
				FVector2f BR = Bounds.Max;
				FVector2f TR = FVector2f(Bounds.Max.X, Bounds.Min.Y);
				FVector2f BL = FVector2f(Bounds.Min.X, Bounds.Max.Y);

				DrawSpaceLine(TL, TR, BoundsWireColor);
				DrawSpaceLine(TR, BR, BoundsWireColor);
				DrawSpaceLine(BR, BL, BoundsWireColor);
				DrawSpaceLine(BL, TL, BoundsWireColor);
			}
#endif
		}

		if (bCloseToSpline)
		{
			// Find the closest approach to the spline
			FVector2f ClosestPoint(ForceInit);
			float ClosestDistanceSquared = FLT_MAX;

			const int32 NumStepsToTest = 16;
			const float StepInterval = 1.0f / (float)NumStepsToTest;
			FVector2f Point1 = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, 0.0f);
			for (float TestAlpha = 0.0f; TestAlpha < 1.0f; TestAlpha += StepInterval)
			{
				const FVector2f Point2 = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, TestAlpha + StepInterval);

				const FVector2f ClosestPointToSegment = FMath::ClosestPointOnSegment2D(AbsoluteMousePosition, Point1, Point2);
				const float DistanceSquared = (AbsoluteMousePosition - ClosestPointToSegment).SizeSquared();

				if (DistanceSquared < ClosestDistanceSquared)
				{
					ClosestDistanceSquared = DistanceSquared;
					ClosestPoint = ClosestPointToSegment;
				}

				Point1 = Point2;
			}

			// Record the overlap
			if (ClosestDistanceSquared < QueryDistanceTriggerThresholdSquared)
			{
				if (ClosestDistanceSquared < SplineOverlapResult.GetDistanceSquared())
				{
					const float SquaredDistToPin1 = (Params.AssociatedPin1 != nullptr) ? (P0 - ClosestPoint).SizeSquared() : FLT_MAX;
					const float SquaredDistToPin2 = (Params.AssociatedPin2 != nullptr) ? (P1 - ClosestPoint).SizeSquared() : FLT_MAX;

					SplineOverlapResult = FGraphSplineOverlapResult(Params.AssociatedPin1, Params.AssociatedPin2, ClosestDistanceSquared, SquaredDistToPin1, SquaredDistToPin2, true);
				}
			}
			else if (ClosestDistanceSquared < QueryDistanceForCloseSquared)
			{
				SplineOverlapResult.SetCloseToSpline(true);
			}
		}
	}

	// Draw the spline itself
	FSlateDrawElement::MakeDrawSpaceSpline(
		DrawElementsList,
		LayerId,
		P0, P0Tangent,
		P1, P1Tangent,
		WireThickness,
		ESlateDrawEffect::None,
		Params.WireColor
	);

	if (Params.bDrawBubbles || (MidpointImage != nullptr))
	{
		// This table maps distance along curve to alpha
		FInterpCurve<float> SplineReparamTable;
		const float SplineLength = MakeSplineReparamTable(P0, P0Tangent, P1, P1Tangent, SplineReparamTable);

		// Draw bubbles on the spline
		if (Params.bDrawBubbles)
		{
			const float BubbleSpacing = 64.f * ZoomFactor;
			const float BubbleSpeed = 192.f * ZoomFactor;
			const FVector2f BubbleSize = BubbleImage->ImageSize * ZoomFactor * 0.2f * Params.WireThickness;

			float Time = (FPlatformTime::Seconds() - GStartTime);
			const float BubbleOffset = FMath::Fmod(Time * BubbleSpeed, BubbleSpacing);
			const int32 NumBubbles = FMath::CeilToInt(SplineLength/BubbleSpacing);
			for (int32 i = 0; i < NumBubbles; ++i)
			{
				const float Distance = ((float)i * BubbleSpacing) + BubbleOffset;
				if (Distance < SplineLength)
				{
					const float Alpha = SplineReparamTable.Eval(Distance, 0.f);
					FVector2f BubblePos = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, Alpha);
					BubblePos -= (BubbleSize * 0.5f);

					FSlateDrawElement::MakeBox(
						DrawElementsList,
						LayerId,
						FPaintGeometry( BubblePos, BubbleSize, ZoomFactor  ),
						BubbleImage,
						ESlateDrawEffect::None,
						Params.WireColor
						);
				}
			}
		}

		// Draw the midpoint image
		if (MidpointImage != nullptr)
		{
			// Determine the spline position for the midpoint
			const float MidpointAlpha = SplineReparamTable.Eval(SplineLength * 0.5f, 0.f);
			const FVector2f Midpoint = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, MidpointAlpha);

			// Approximate the slope at the midpoint (to orient the midpoint image to the spline)
			const FVector2f MidpointPlusE = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, MidpointAlpha + KINDA_SMALL_NUMBER);
			const FVector2f MidpointMinusE = FMath::CubicInterp(P0, P0Tangent, P1, P1Tangent, MidpointAlpha - KINDA_SMALL_NUMBER);
			const FVector2f SlopeUnnormalized = MidpointPlusE - MidpointMinusE;

			// Draw the arrow
			const FVector2f MidpointDrawPos = Midpoint - MidpointRadius;
			const float AngleInRadians = SlopeUnnormalized.IsNearlyZero() ? 0.0f : FMath::Atan2(SlopeUnnormalized.Y, SlopeUnnormalized.X);

			FSlateDrawElement::MakeRotatedBox(
				DrawElementsList,
				LayerId,
				FPaintGeometry(MidpointDrawPos, MidpointImage->ImageSize * ZoomFactor, ZoomFactor),
				MidpointImage,
				ESlateDrawEffect::None,
				AngleInRadians,
				TOptional<FVector2f>(),
				FSlateDrawElement::RelativeToElement,
				Params.WireColor
				);
		}
	}

}