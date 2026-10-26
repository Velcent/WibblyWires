// Copyright 2022 Geordie Hall. All rights reserved.

#include "WibblyWires.h"

#include "BlueprintEditorModule.h"
#include "EdGraphUtilities.h"
#include "WibblyConnectionDrawingPolicy.h"
#include "Framework/Notifications/NotificationManager.h"
#include "Widgets/Notifications/SNotificationList.h"
#include "BlueprintEditorModule.h"
#include "Framework/Application/SlateApplication.h"

#define LOCTEXT_NAMESPACE "FWibblyWiresModule"

static TSharedPtr<FWibblyConnectionDrawingPolicy::Factory> GraphConnectionFactory;
static TSharedPtr<FWibblyCutInputProcessor> WibblyCutProcessor;

void FWibblyWiresModule::StartupModule()
{
	GraphConnectionFactory = MakeShared<FWibblyConnectionDrawingPolicy::Factory>();
	FEdGraphUtilities::RegisterVisualPinConnectionFactory(GraphConnectionFactory);

	WibblyCutProcessor = MakeShared<FWibblyCutInputProcessor>();
	FSlateApplication::Get().RegisterInputPreProcessor(WibblyCutProcessor);
}

void FWibblyWiresModule::ShutdownModule()
{
	if (GraphConnectionFactory.IsValid())
	{
		FEdGraphUtilities::UnregisterVisualPinConnectionFactory(GraphConnectionFactory);
		GraphConnectionFactory = nullptr;
	}

	if (FSlateApplication::IsInitialized() && WibblyCutProcessor.IsValid())
	{
		FSlateApplication::Get().UnregisterInputPreProcessor(WibblyCutProcessor);
	}
	WibblyCutProcessor.Reset();
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FWibblyWiresModule, WibblyWires)