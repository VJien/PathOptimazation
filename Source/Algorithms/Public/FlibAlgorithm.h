// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "FlibAlgorithm.generated.h"


/**
 * 优化算法区别
 假设计算的点为key，key前置的向量为DirFWD，后置的向量是DirBWD
 以下3个类型针对的是前置向量的计算方式
 */
UENUM(BlueprintType)
enum class ECurvatureType :uint8
{
	T124,
	T134,
	T234
};


/**
 *
 */
UCLASS()
class ALGORITHMS_API UFlibAlgorithm : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
		//径向距离优化，小于distance的点会被优化
		UFUNCTION(BlueprintCallable, Category = "UFlibAlgorithm|PathPointOptimization")
		static bool RadialDistanceOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, float Distance = 50);
	//垂直距离优化，大于distance的点会被优化
	UFUNCTION(BlueprintCallable, Category = "UFlibAlgorithm|PathPointOptimization")
		static bool PerpendicularDistanceOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, float Distance = 50);
	//曲率优化，大于角度angle的点会被优化，分为3个模式，推荐使用T134
	UFUNCTION(BlueprintCallable, Category = "UFlibAlgorithm|PathPointOptimization")
		static bool CurvatureOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, ECurvatureType type = ECurvatureType::T134, float Angle = 90);
	/*
	达格拉斯多普优化，使用迭代计算
	效率相对低但是精度高
	*/
	UFUNCTION(BlueprintCallable, Category = "UFlibAlgorithm|PathPointOptimization")
		static bool DouglasPeuckerOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, float Distance = 50);


	UFUNCTION(BlueprintCallable)
		static float AngleBetween(FVector A, FVector B);


};
