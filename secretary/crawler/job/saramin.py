import requests
from bs4 import BeautifulSoup
import pandas as pd
import time
import math
import re
import os

# 기본 설정
base_url = "https://www.saramin.co.kr/zf_user/search/recruit"
search_keywords = ["인공지능", "로봇"]
job_list = []

# User-Agent 설정
headers = {
    "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/96.0.4664.110 Safari/537.36"
}

# 제외할 키워드 (웹개발 관련)
exclude_keywords = ["웹개발", "프론트엔드", "백엔드", "풀스택", "JavaScript", "React", "Vue"]

# 검색어별로 크롤링
for keyword in search_keywords:
    params = {
        "searchword": keyword,
        "recruitPage": 1,
        "recruitSort": "relation",
        "recruitPageCount": 50,
        "loc_cd": 101000,  # 지역 코드: 서울
    }

    print(f"\n=== Starting crawl for keyword: {keyword} ===")

    # 1. 총 공고 수 추출
    try:
        response = requests.get(base_url, params=params, headers=headers)
        soup = BeautifulSoup(response.text, "html.parser")

        # <title>에서 총 공고 수 추출
        title_tag = soup.find("title").get_text(strip=True)
        total_jobs = int(title_tag.split("총 ")[1].split("건")[0].replace(",", ""))
        total_pages = math.ceil(total_jobs / params["recruitPageCount"])
        print(f"Keyword '{keyword}' has {total_jobs} jobs across {total_pages} pages.")

    except Exception as e:
        print(f"Failed to fetch total job count for keyword {keyword}: {e}")
        continue

    # 2. 페이지별 크롤링
    while params["recruitPage"] <= total_pages:
        try:
            response = requests.get(base_url, params=params, headers=headers)
            print(f"Fetching page {params['recruitPage']} for keyword: {keyword}")

            if response.status_code != 200:
                print(f"Failed to fetch data for keyword: {keyword}, page: {params['recruitPage']}")
                break

            # HTML 파싱
            soup = BeautifulSoup(response.text, "html.parser")
            job_items = soup.find_all("div", class_="item_recruit")
            print(f"Found {len(job_items)} jobs on page {params['recruitPage']} for keyword: {keyword}")

            if not job_items:
                print("No job items found on this page. HTML structure may have changed.")
                break

            for job in job_items:
                try:
                    # 제목 추출
                    title_tag = job.find("h2", class_="job_tit")
                    title = title_tag.get_text(strip=True) if title_tag else "N/A"

                    # 회사명 추출
                    company_tag = job.find("strong", class_="corp_name")
                    company = company_tag.get_text(strip=True) if company_tag else "N/A"

                    # 링크 추출
                    link_tag = title_tag.find("a") if title_tag else None
                    link = link_tag["href"] if link_tag else "#"

                    # 위치 추출
                    location_tag = job.find("div", class_="job_condition")
                    location = location_tag.get_text(strip=True) if location_tag else "N/A"

                    # 직무 설명 (주업무)
                    job_description_tag = job.find("div", class_="job_sector")
                    job_description = job_description_tag.get_text(strip=True) if job_description_tag else "N/A"

                    # 연봉 정보
                    salary_tag = job.find("div", class_="salary")
                    salary = salary_tag.get_text(strip=True) if salary_tag else "N/A"

                    # 조건에 맞는 데이터만 추가
                    if any(re.search(kw, title, re.IGNORECASE) for kw in exclude_keywords):
                        print(f"Excluded job (Web Development): {title}")
                        continue

                    if "연구" not in title and "연구" not in company:
                        print(f"Excluded job (No '연구'): {title}")
                        continue

                    job_list.append({
                        "Keyword": keyword,
                        "Title": title,
                        "Company": company,
                        "Location": location,
                        "Job Description": job_description,
                        "Salary": salary,
                        "Link": f"https://www.saramin.co.kr{link}"
                    })
                    print(f"Extracted job: {title}, {company}, {location}, {job_description}, {salary}")

                except Exception as e:
                    print(f"Error extracting job details: {e}")

            # 페이지 증가
            params["recruitPage"] += 1
            time.sleep(1)

        except Exception as e:
            print(f"Error fetching data: {e}")
            break

save_path = os.path.join("crawler", "job", "data", "saramin_filtered_research_jobs.csv")

print("\n=== Saving results ===")
try:
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    df = pd.DataFrame(job_list)
    df.to_csv(save_path, index=False, encoding="utf-8-sig")
    print(f"Saved {len(job_list)} jobs to {save_path}")
except Exception as e:
    print(f"Error saving data to CSV: {e}")
finally:
    print(f"크롤링 완료. 총 {len(job_list)}개의 공고를 저장했습니다.")
