// Fill out your copyright notice in the Description page of Project Settings.


#include "FlibAlgorithm.h"
#include "Kismet/KismetMathLibrary.h"

bool UFlibAlgorithm::RadialDistanceOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, float Distance)
{
	if (InPath.Num() < 3)
	{
		OutPath = InPath;
		return false;
	}
	FVector vKey = InPath[0];
	OutPath.Add(vKey);
	for (int32 i = 1; i < InPath.Num() - 1; i++)
	{
		if ((vKey - InPath[i]).Size() > Distance)
		{
			OutPath.Add(InPath[i]);
			vKey = InPath[i];
		}
	}
	if ((InPath.Last() - vKey).Size() <= Distance)
	{
		OutPath.Pop();
	}
	OutPath.Add(InPath.Last());
	return true;
}

bool UFlibAlgorithm::PerpendicularDistanceOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, float Distance /*= 50*/)
{
	if (InPath.Num() < 3)
	{
		OutPath = InPath;
		return false;
	}
	FVector vKey = InPath[0];
	OutPath.Add(vKey);
	FVector p1, p2, p3;
	p1 = InPath[0];
	for (int32 i = 1; i < InPath.Num() - 1; i++)
	{
		p2 = InPath[i];
		p3 = InPath[i + 1];
		float dis = UKismetMathLibrary::GetPointDistanceToLine(p2, p1, p3 - p1);
		if (dis <= Distance)
		{
			OutPath.Add(p2);
			p1 = p2;
		}
	}
	OutPath.Add(InPath.Last());
	return true;

}

bool UFlibAlgorithm::CurvatureOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, ECurvatureType type, float Angle /*= 90*/)
{
	if (InPath.Num() < 3)
	{
		OutPath = InPath;
		return false;
	}
	FVector p1 = InPath[0];
	OutPath.Add(p1);
	FVector p2, p3, dirFWD, dirBWD;
	bool b124Found = true;
	for (int32 i = 1; i < InPath.Num() - 1; i++)
	{
		p2 = InPath[i];
		p3 = InPath[i + 1];
		if (b124Found || type != ECurvatureType::T124)
		{
			dirFWD = p2 - p1;
		}
		dirBWD = p3 - p2;
		if (AngleBetween(dirBWD, dirFWD) <= Angle)
		{
			OutPath.Add(p2);
			p1 = p2;
			b124Found = true;
		}
		else
		{
			b124Found = false;
			if (type == ECurvatureType::T234)
			{
				p1 = p2;
			}
		}


	}
	OutPath.Add(InPath.Last());
	return true;
}

bool UFlibAlgorithm::DouglasPeuckerOptimization(const TArray<FVector>& InPath, TArray<FVector>& OutPath, float Distance /*= 50*/)
{

	if (InPath.Num() < 3)
	{
		OutPath = InPath;
		return false;
	}
	int32 idx, first, last;
	first = idx = 0;
	last = InPath.Num() - 1;
	TArray<bool> Markers;
	Markers.AddUninitialized(last + 1);
	Markers[0] = true;
	Markers.Last() = true;
	float maxDis = 0;
	TArray<int32> stack;
	while (first >= 0 && last >= 0)
	{
		maxDis = 0;
		for (int32 i = first + 1; i < last; i++)
		{
			float dis = UKismetMathLibrary::GetPointDistanceToLine(InPath[i], InPath[first], InPath[last] - InPath[first]);
			if (dis > maxDis)
			{
				idx = i;
				maxDis = dis;
			}
		}
		if (maxDis > Distance)
		{
			Markers[idx] = true;
			stack.Add(first);
			stack.Add(idx);
			stack.Add(idx);
			stack.Add(last);
		}
		if (stack.Num() > 0)
		{
			last = stack.Pop();
		}
		else
		{
			last = -1;
		}
		if (stack.Num() > 0)
		{
			first = stack.Pop();
		}
		else
		{
			first = -1;
		}

	}

	for (int32 i = 0; i < Markers.Num(); i++)
	{
		if (Markers[i])
		{
			OutPath.Add(InPath[i]);
		}
	}
	return true;



}

float UFlibAlgorithm::AngleBetween(FVector A, FVector B)
{
	A.Normalize();
	B.Normalize();
	return UKismetMathLibrary::DegAcos(UKismetMathLibrary::Dot_VectorVector(A, B));

}
